#!/usr/bin/env python3
"""Fast native ROS 2/RViz replay of the point cloud, camera, frustum, and LZs."""

from __future__ import annotations

import argparse
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Point
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

SCRIPTS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPTS))

from lz_bag_replay_viewer.replay_server import (  # noqa: E402
    camera_footprint, displayed_zone_ring, orthometric_from_map,
)
from lz_camera_projection import compose, load_landing_zones, scaled_camera_matrix, transform_points  # noqa: E402
from project_lz_into_bag_camera import (  # noqa: E402
    chain_transform, config_transform, decode_image, draw_zones, fit_utm_to_map,
    open_reader, read_pose_and_gps, require_ros, stamp_ns,
)

FRAME_ID = "lz_world"
COLORS = {
    "safest": (0.0, 0.50, 0.0, 1.0),
    "safe": (1.0, 0.647, 0.0, 1.0),
    "risky": (1.0, 0.0, 0.0, 1.0),
    "top30": (0.565, 0.933, 0.565, 1.0),
    "pink": (1.0, 0.714, 0.757, 1.0),
}


def ros_stamp(message: object, timestamp_ns: int) -> None:
    message.header.stamp.sec = timestamp_ns // 1_000_000_000
    message.header.stamp.nanosec = timestamp_ns % 1_000_000_000


def read_ascii_pcd(path: Path, max_points: int) -> np.ndarray:
    with path.open("r", encoding="ascii") as handle:
        while True:
            line = handle.readline()
            if not line:
                raise ValueError(f"PCD has no DATA header: {path}")
            if line.strip().lower() == "data ascii":
                break
        points = np.loadtxt(handle, dtype=np.float64, usecols=(0, 1, 2))
    if len(points) > max_points:
        indices = np.random.default_rng(17).choice(len(points), max_points, replace=False)
        points = points[indices]
    return points


def point_cloud_message(points: np.ndarray, origin: np.ndarray) -> PointCloud2:
    local = np.ascontiguousarray(points - origin, dtype=np.float32)
    message = PointCloud2()
    message.header = Header(frame_id=FRAME_ID)
    message.height = 1
    message.width = len(local)
    message.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    message.is_bigendian = False
    message.point_step = 12
    message.row_step = message.point_step * message.width
    message.is_dense = bool(np.isfinite(local).all())
    message.data = local.tobytes()
    return message


def marker(marker_id: int, namespace: str, marker_type: int, color: tuple[float, ...],
           width: float, timestamp_ns: int) -> Marker:
    item = Marker()
    item.header.frame_id = FRAME_ID
    ros_stamp(item, timestamp_ns)
    item.ns = namespace
    item.id = marker_id
    item.type = marker_type
    item.action = Marker.ADD
    item.pose.orientation.w = 1.0
    item.scale.x = width
    item.scale.y = width
    item.scale.z = width
    item.color.r, item.color.g, item.color.b, item.color.a = color
    return item


def append_segments(item: Marker, points: np.ndarray, origin: np.ndarray, close: bool = True) -> None:
    local = points - origin
    stop = len(local) if close else len(local) - 1
    for index in range(stop):
        first = local[index]
        second = local[(index + 1) % len(local)]
        item.points.extend((Point(x=float(first[0]), y=float(first[1]), z=float(first[2])),
                            Point(x=float(second[0]), y=float(second[1]), z=float(second[2]))))


class LandingZoneRvizReplay(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("landing_zone_rviz_replay")
        self.args = args
        self.stop = threading.Event()
        with args.config.open(encoding="utf-8") as handle:
            self.config = yaml.safe_load(handle)
        self.zones = load_landing_zones(args.zones, None, 0)
        cloud = read_ascii_pcd(args.pcd, args.max_points)
        self.origin = np.array([float(np.median(cloud[:, 0])), float(np.median(cloud[:, 1])), 0.0])

        durable = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        live = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.cloud_pub = self.create_publisher(PointCloud2, "/ddlzd/pointcloud", durable)
        self.static_pub = self.create_publisher(MarkerArray, "/ddlzd/landing_zones", durable)
        self.dynamic_pub = self.create_publisher(MarkerArray, "/ddlzd/replay_markers", live)
        self.image_pub = self.create_publisher(Image, "/ddlzd/camera_overlay", live)
        self.cloud_pub.publish(point_cloud_message(cloud, self.origin))
        self.static_pub.publish(self.static_zone_markers())
        self.get_logger().info(f"Published {len(cloud)} cloud points and {len(self.zones)} landing zones")
        self.worker = threading.Thread(target=self.replay, daemon=True)
        self.worker.start()

    def static_zone_markers(self) -> MarkerArray:
        grouped = {
            name: marker(index, f"landing_zones_{name}", Marker.LINE_LIST, COLORS[name], 0.07, 0)
            for index, name in enumerate(("top30", "safest", "safe", "risky"))
        }
        for zone in self.zones:
            name = "top30" if int(zone["rank"]) <= 30 else str(zone["category"])
            append_segments(grouped[name], displayed_zone_ring(zone, samples=32), self.origin)
        return MarkerArray(markers=list(grouped.values()))

    def dynamic_markers(self, timestamp_ns: int, visible_zones: list[dict], trajectory: list[np.ndarray],
                        uav: np.ndarray, camera: np.ndarray, footprint: list[list[float]]) -> MarkerArray:
        clear = Marker()
        clear.header.frame_id = FRAME_ID
        ros_stamp(clear, timestamp_ns)
        clear.action = Marker.DELETEALL

        pink = marker(0, "camera_visible_lz", Marker.LINE_LIST, COLORS["pink"], 0.18, timestamp_ns)
        for zone in visible_zones:
            append_segments(pink, displayed_zone_ring(zone, samples=32), self.origin)

        trail = marker(1, "uav_trajectory", Marker.LINE_STRIP, (0.16, 0.71, 0.96, 1.0), 0.12, timestamp_ns)
        for value in trajectory:
            local = value - self.origin
            trail.points.append(Point(x=float(local[0]), y=float(local[1]), z=float(local[2])))

        aircraft = marker(2, "uav", Marker.SPHERE, (1.0, 1.0, 1.0, 1.0), 0.55, timestamp_ns)
        aircraft.pose.position = Point(x=float(uav[0] - self.origin[0]),
                                       y=float(uav[1] - self.origin[1]), z=float(uav[2]))

        outline = marker(3, "camera_footprint", Marker.LINE_STRIP, (0.0, 0.90, 1.0, 1.0), 0.13, timestamp_ns)
        for value in footprint:
            local = np.asarray(value) - self.origin
            outline.points.append(Point(x=float(local[0]), y=float(local[1]), z=float(local[2])))

        rays = marker(4, "camera_frustum", Marker.LINE_LIST, (0.50, 0.87, 0.92, 1.0), 0.07, timestamp_ns)
        local_camera = camera - self.origin
        for value in footprint[:-1]:
            local = np.asarray(value) - self.origin
            rays.points.extend((Point(x=float(local_camera[0]), y=float(local_camera[1]), z=float(local_camera[2])),
                                Point(x=float(local[0]), y=float(local[1]), z=float(local[2]))))
        return MarkerArray(markers=[clear, pink, trail, aircraft, outline, rays])

    def publish_image(self, frame: np.ndarray, timestamp_ns: int) -> None:
        if self.args.image_width > 0 and frame.shape[1] != self.args.image_width:
            height = int(round(frame.shape[0] * self.args.image_width / frame.shape[1]))
            frame = cv2.resize(frame, (self.args.image_width, height), interpolation=cv2.INTER_AREA)
        message = Image()
        message.header.frame_id = "camera_optical"
        ros_stamp(message, timestamp_ns)
        message.height, message.width = frame.shape[:2]
        message.encoding = "bgr8"
        message.is_bigendian = False
        message.step = message.width * 3
        message.data = np.ascontiguousarray(frame).tobytes()
        self.image_pub.publish(message)

    def replay(self) -> None:
        try:
            tf_data, gps = read_pose_and_gps(self.args.bag, self.config)
            utm_to_map, diagnostics = fit_utm_to_map(tf_data, gps, self.config)
            vertical_offset = float(self.config["registration"]["lz_to_gnss_vertical_offset_m"])
            map_to_ortho = orthometric_from_map(utm_to_map, vertical_offset)
            ground_z = float(np.median([float(zone["z"]) for zone in self.zones]))
            zones_by_rank = {int(zone["rank"]): zone for zone in self.zones}
            self.get_logger().info(
                f"Registration ready: RMSE {diagnostics['horizontal_rmse_m']:.3f} m; replay starting"
            )

            _, deserialize_message, get_message = require_ros()
            reader = open_reader(self.args.bag)
            topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
            image_topic = self.config["topics"]["image"]
            image_type = get_message(topic_types[image_topic])
            native_k = np.asarray(self.config["camera"]["matrix"], dtype=np.float64)
            distortion = np.asarray(self.config["camera"]["distortion"], dtype=np.float64)
            first_stamp = last_publish = None
            wall_start = time.monotonic()
            trajectory: list[np.ndarray] = []
            frame_number = 0
            interval_ns = int(1e9 / self.args.fps)
            while rclpy.ok() and not self.stop.is_set() and reader.has_next():
                topic, raw, bag_time = reader.read_next()
                if topic != image_topic:
                    continue
                message = deserialize_message(raw, image_type)
                timestamp = stamp_ns(message, bag_time)
                if first_stamp is None:
                    first_stamp = timestamp
                if last_publish is not None and timestamp - last_publish < interval_ns:
                    continue
                desired = (timestamp - first_stamp) / 1e9 / self.args.rate
                delay = desired - (time.monotonic() - wall_start)
                if delay > 0:
                    time.sleep(delay)

                frame = decode_image(message)
                camera_matrix = scaled_camera_matrix(
                    native_k, tuple(self.config["camera"]["calibration_size"]),
                    (frame.shape[1], frame.shape[0]),
                )
                map_to_mount = chain_transform(
                    tf_data, self.config["frames"]["camera_chain"], timestamp,
                    float(self.config["synchronization"]["max_tf_gap_s"]),
                )
                map_to_optical = compose(map_to_mount, config_transform(self.config["camera"]["mount_to_optical"]))
                ranks: list[int] = []
                draw_zones(frame, self.zones, map_to_optical, utm_to_map, camera_matrix,
                           distortion, vertical_offset, 0.25, visible_ranks_out=ranks)
                ortho_to_optical = compose(map_to_ortho, map_to_optical)
                map_to_base = chain_transform(
                    tf_data, [self.config["frames"]["map"], self.config["frames"]["base"]], timestamp,
                    float(self.config["synchronization"]["max_tf_gap_s"]),
                )
                uav = transform_points(map_to_ortho, map_to_base.translation[None, :])[0]
                if not trajectory or np.linalg.norm(uav - trajectory[-1]) > 0.1:
                    trajectory.append(uav)
                footprint, _ = camera_footprint(
                    ortho_to_optical, camera_matrix, distortion, frame.shape[1], frame.shape[0], ground_z,
                )
                frame_number += 1
                relative = (timestamp - first_stamp) / 1e9
                text = f"Frame {frame_number:06d} | t={relative:.3f}s | ROS {timestamp} ns"
                cv2.rectangle(frame, (0, 0), (frame.shape[1], 42), (0, 0, 0), -1)
                cv2.putText(frame, text, (14, 29), cv2.FONT_HERSHEY_SIMPLEX,
                            0.68, (255, 255, 255), 2, cv2.LINE_AA)
                self.dynamic_pub.publish(self.dynamic_markers(
                    timestamp, [zones_by_rank[rank] for rank in ranks], trajectory,
                    uav, ortho_to_optical.translation, footprint,
                ))
                self.publish_image(frame, timestamp)
                last_publish = timestamp
                if frame_number == 1 or frame_number % 120 == 0:
                    self.get_logger().info(
                        f"Published frame {frame_number} at bag t={relative:.2f}s with {len(ranks)} visible LZs"
                    )
            self.get_logger().info("Replay complete")
        except Exception as exc:
            if rclpy.ok() and not self.stop.is_set():
                self.get_logger().error(f"Replay failed: {exc}")


def main() -> int:
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--pcd", type=Path, default=root / "pcds/696_5328_cropped_by_bayernatlas_20260812084517.pcd")
    parser.add_argument("--zones", type=Path, default=root / "pcds/region_growing_landing_zones_max10000_radius2_5_to3_risk_ranked.csv")
    parser.add_argument("--config", type=Path, default=root / "scripts/config/lz_camera_projection.yaml")
    parser.add_argument("--max-points", type=int, default=250000)
    parser.add_argument("--fps", type=float, default=12.0)
    parser.add_argument("--rate", type=float, default=1.0)
    parser.add_argument("--image-width", type=int, default=640)
    args = parser.parse_args()
    rclpy.init()
    node = LandingZoneRvizReplay(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop.set()
        if rclpy.ok():
            rclpy.shutdown()
        node.worker.join(timeout=5.0)
        node.destroy_node()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
