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
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray

SCRIPTS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPTS))

from lz_bag_replay_viewer.replay_server import (  # noqa: E402
    camera_footprint, displayed_zone_ring, orthometric_from_map,
)
from lz_camera_projection import compose, load_landing_zones, scaled_camera_matrix, transform_points  # noqa: E402
from lz_rviz_viewer.dynamic_occupancy import (  # noqa: E402
    apply_centered_transform, DynamicOccupancyMap, interpolate_gps, pointcloud2_xyz,
    TerrainHeightModel,
)
from lz_rviz_viewer.validated_pose_replay import ValidatedPoseReplay  # noqa: E402
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
    "blocked": (0.55, 0.55, 0.58, 0.95),
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
    if max_points > 0 and len(points) > max_points:
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
        full_cloud = read_ascii_pcd(args.pcd, 0)
        ground_cloud = read_ascii_pcd(args.ground_pcd, 0)
        self.terrain = TerrainHeightModel(ground_cloud)
        cloud = full_cloud
        if len(cloud) > args.max_points:
            indices = np.random.default_rng(17).choice(len(cloud), args.max_points, replace=False)
            cloud = cloud[indices]
        self.lidar_registration_origin = np.median(full_cloud, axis=0)
        self.origin = self.lidar_registration_origin.copy()
        # Keep native elevation values in RViz while using the true 3D median as
        # the rotation pivot for the ICP correction.
        self.origin[2] = 0.0
        self.lidar_correction = config_transform(self.config["registration"]["live_lidar_to_static"])
        dynamic = self.config["dynamic_occupancy"]
        validated = dynamic.get("validated_pose_replay", {})
        self.validated_replay = None
        if bool(validated.get("enabled", False)):
            if not args.optimized_poses.is_file():
                raise FileNotFoundError(
                    f"Validated pose replay is enabled but poses are missing: {args.optimized_poses}"
                )
            with args.mapping_config.open(encoding="utf-8") as handle:
                mapping_config = yaml.safe_load(handle)
            self.validated_replay = ValidatedPoseReplay(
                args.optimized_poses, mapping_config["extrinsic_base_from_lidar"],
                maximum_time_error_s=float(validated["maximum_timestamp_error_s"]),
            )
        self.occupancy = DynamicOccupancyMap(
            full_cloud, self.zones,
            resolution=float(dynamic["resolution_m"]),
            gps_margin_m=float(dynamic["gps_wind_margin_m"]),
            obstacle_min_height_m=float(dynamic["obstacle_min_height_m"]),
            vertical_clearance_m=float(dynamic["vertical_clearance_m"]),
            min_obstacle_voxels=int(dynamic["confirmation_hits"]),
            coarse_resolution=float(dynamic["coarse_resolution_m"]),
            interest_padding_m=float(dynamic["interest_padding_m"]),
        )
        self.active_zones, self.blocked_zones = self.occupancy.classified_zones()

        durable = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        live = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.cloud_pub = self.create_publisher(PointCloud2, "/ddlzd/pointcloud", durable)
        self.static_pub = self.create_publisher(MarkerArray, "/ddlzd/landing_zones", durable)
        self.dynamic_pub = self.create_publisher(MarkerArray, "/ddlzd/replay_markers", live)
        self.occupancy_pub = self.create_publisher(MarkerArray, "/ddlzd/occupancy", durable)
        self.image_pub = self.create_publisher(Image, "/ddlzd/camera_overlay", live)
        self.cloud_pub.publish(point_cloud_message(cloud, self.origin))
        self.static_pub.publish(self.zone_markers(0))
        self.occupancy_pub.publish(self.occupancy_markers(0, include_baseline=True))
        self.get_logger().info(
            f"Published {len(cloud)} cloud points, {len(self.zones)} landing zones, and "
            f"{self.occupancy.baseline_voxel_count} adaptive baseline occupancy voxels"
        )
        self.worker = threading.Thread(target=self.replay, daemon=True)
        self.worker.start()

    def zone_markers(self, timestamp_ns: int) -> MarkerArray:
        clear = Marker()
        clear.header.frame_id = FRAME_ID
        ros_stamp(clear, timestamp_ns)
        clear.action = Marker.DELETEALL
        grouped = {
            name: marker(index, f"landing_zones_{name}", Marker.LINE_LIST, COLORS[name], 0.07, timestamp_ns)
            for index, name in enumerate(("top30", "safest", "safe", "risky"))
        }
        for zone in self.active_zones:
            name = "top30" if int(zone["rank"]) <= 30 and zone["dynamic_state"] == "unchanged" else str(zone["category"])
            append_segments(grouped[name], self.visual_zone_ring(zone), self.origin)
        blocked = marker(10, "landing_zones_blocked", Marker.LINE_LIST, COLORS["blocked"], 0.14, timestamp_ns)
        for zone in self.blocked_zones:
            center = np.array([
                float(zone["x"]), float(zone["y"]),
                float(zone["z"]) + float(self.config["dynamic_occupancy"]["zone_visual_lift_m"]),
            ])
            radius = float(zone["radius"])
            segments = np.array([
                center + [-radius, -radius, 0.0], center + [radius, radius, 0.0],
                center + [-radius, radius, 0.0], center + [radius, -radius, 0.0],
            ])
            append_segments(blocked, segments[:2], self.origin, close=False)
            append_segments(blocked, segments[2:], self.origin, close=False)
        return MarkerArray(markers=[clear, *grouped.values(), blocked])

    def visual_zone_ring(self, zone: dict) -> np.ndarray:
        ring = displayed_zone_ring(zone, samples=32)
        ring[:, 2] += float(self.config["dynamic_occupancy"]["zone_visual_lift_m"])
        return ring

    @staticmethod
    def altitude_color(value: float) -> ColorRGBA:
        value = float(np.clip(value, 0.0, 1.0))
        anchors = np.array([[0.08, 0.20, 0.72], [0.00, 0.78, 0.88],
                            [0.95, 0.88, 0.15], [0.88, 0.12, 0.08]])
        scaled = value * (len(anchors) - 1)
        index = min(int(scaled), len(anchors) - 2)
        fraction = scaled - index
        rgb = anchors[index] * (1.0 - fraction) + anchors[index + 1] * fraction
        return ColorRGBA(r=float(rgb[0]), g=float(rgb[1]), b=float(rgb[2]), a=0.46)

    def occupancy_markers(self, timestamp_ns: int, include_baseline: bool = False) -> MarkerArray:
        limit = int(self.config["dynamic_occupancy"]["max_visualized_voxels"])
        layers = self.occupancy.occupancy_layers(include_baseline=include_baseline)
        total = max(1, sum(len(values) for _, values, _ in layers))
        output: list[Marker] = []
        for layer_index, (level, values, resolution) in enumerate(layers):
            layer_limit = max(1, int(limit * len(values) / total))
            if len(values) > layer_limit:
                indices = np.random.default_rng(23 + layer_index).choice(
                    len(values), layer_limit, replace=False,
                )
                values = values[indices]
            marker_id = (0 if include_baseline else 10) + layer_index
            prefix = "baseline_occupancy" if include_baseline else "live_occupancy"
            item = marker(marker_id, f"{prefix}_{level}", Marker.CUBE_LIST,
                          (1.0, 1.0, 1.0, 0.72), resolution * 0.92, timestamp_ns)
            if not len(values):
                output.append(item)
                continue
            z_min, z_max = float(values[:, 2].min()), float(values[:, 2].max())
            span = max(z_max - z_min, 1e-6)
            for value in values:
                local = value - self.origin
                item.points.append(Point(x=float(local[0]), y=float(local[1]), z=float(local[2])))
                item.colors.append(self.altitude_color((float(value[2]) - z_min) / span))
            output.append(item)
        return MarkerArray(markers=output)

    def dynamic_markers(self, timestamp_ns: int, visible_zones: list[dict], trajectory: list[np.ndarray],
                        uav: np.ndarray, camera: np.ndarray, footprint: list[list[float]],
                        frame_number: int, tf_uav: np.ndarray) -> MarkerArray:
        clear = Marker()
        clear.header.frame_id = FRAME_ID
        ros_stamp(clear, timestamp_ns)
        clear.action = Marker.DELETEALL

        pink = marker(0, "camera_visible_lz", Marker.LINE_LIST, COLORS["pink"], 0.18, timestamp_ns)
        for zone in visible_zones:
            append_segments(pink, self.visual_zone_ring(zone), self.origin)

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

        label = marker(5, "replay_sync", Marker.TEXT_VIEW_FACING, (1.0, 1.0, 1.0, 1.0), 0.75, timestamp_ns)
        label.pose.position = Point(x=float(uav[0] - self.origin[0]),
                                    y=float(uav[1] - self.origin[1]), z=float(uav[2] + 1.2))
        label.text = f"Frame {frame_number:06d} | {timestamp_ns} ns"

        registration = marker(6, "geographic_registration", Marker.LINE_LIST,
                              (1.0, 0.2, 0.8, 1.0), 0.09, timestamp_ns)
        for value in (uav, tf_uav):
            local = value - self.origin
            registration.points.append(Point(x=float(local[0]), y=float(local[1]), z=float(local[2])))
        return MarkerArray(markers=[clear, pink, trail, aircraft, outline, rays, label, registration])

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
            gps_times = np.fromiter((item[0] for item in gps), dtype=np.int64)
            map_to_ortho = orthometric_from_map(utm_to_map, vertical_offset)
            ground_z = float(np.median([float(zone["z"]) for zone in self.zones]))
            self.get_logger().info(
                f"Registration ready: RMSE {diagnostics['horizontal_rmse_m']:.3f} m; replay starting"
            )

            _, deserialize_message, get_message = require_ros()
            reader = open_reader(self.args.bag)
            topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
            image_topic = self.config["topics"]["image"]
            lidar_topic = self.config["topics"]["lidar_points"]
            image_type = get_message(topic_types[image_topic])
            lidar_type = get_message(topic_types[lidar_topic])
            native_k = np.asarray(self.config["camera"]["matrix"], dtype=np.float64)
            distortion = np.asarray(self.config["camera"]["distortion"], dtype=np.float64)
            first_stamp = last_publish = None
            last_occupancy_publish = None
            last_zone_publish = None
            zones_dirty = False
            wall_start = time.monotonic()
            trajectory: list[np.ndarray] = []
            frame_number = 0
            lidar_frame_number = 0
            pose_index = -1
            pose_time_error = 0.0
            interval_ns = int(1e9 / self.args.fps)
            occupancy_interval_ns = int(1e9 / float(self.config["dynamic_occupancy"]["publish_hz"]))
            zone_interval_ns = int(1e9 / float(self.config["dynamic_occupancy"]["zone_publish_hz"]))
            max_lidar_range = float(self.config["dynamic_occupancy"]["maximum_lidar_range_m"])
            lidar_stride = int(self.config["dynamic_occupancy"]["point_stride"])
            corrected_alignment_ready = False
            while rclpy.ok() and not self.stop.is_set() and reader.has_next():
                topic, raw, bag_time = reader.read_next()
                if topic == lidar_topic:
                    message = deserialize_message(raw, lidar_type)
                    timestamp = stamp_ns(message, bag_time)
                    try:
                        if self.validated_replay is not None:
                            if not corrected_alignment_ready:
                                map_to_base = chain_transform(
                                    tf_data, [self.config["frames"]["map"], self.config["frames"]["base"]],
                                    timestamp, float(self.config["synchronization"]["max_tf_gap_s"]),
                                )
                                centered = np.eye(4)
                                centered[:3, 3] = self.lidar_registration_origin
                                uncentered = np.eye(4)
                                uncentered[:3, 3] = -self.lidar_registration_origin
                                static_correction = centered @ self.lidar_correction.matrix @ uncentered
                                world_from_body = static_correction @ map_to_ortho.matrix @ map_to_base.matrix
                                self.validated_replay.initialize_world_alignment(world_from_body)
                                corrected_alignment_ready = True
                            ortho_points, pose_index, pose_time_error = self.validated_replay.transform(
                                message, bag_time, lidar_stride, 1.0, max_lidar_range,
                            )
                        else:
                            lidar_points = pointcloud2_xyz(message)[::lidar_stride]
                            ranges = np.linalg.norm(lidar_points, axis=1)
                            lidar_points = lidar_points[(ranges > 1.0) & (ranges <= max_lidar_range)]
                            map_to_lidar = chain_transform(
                                tf_data, self.config["frames"]["lidar_chain"], timestamp,
                                float(self.config["synchronization"]["max_tf_gap_s"]),
                            )
                            map_points = transform_points(map_to_lidar, lidar_points)
                            ortho_points = transform_points(map_to_ortho, map_points)
                            ortho_points = apply_centered_transform(
                                ortho_points, self.lidar_correction.matrix, self.lidar_registration_origin,
                            )
                            pose_index, pose_time_error = lidar_frame_number, 0.0
                    except LookupError as error:
                        self.get_logger().warning(str(error))
                        continue
                    changed = self.occupancy.update(ortho_points)
                    lidar_frame_number += 1
                    if changed:
                        self.active_zones, self.blocked_zones = self.occupancy.classified_zones()
                        zones_dirty = True
                    if (zones_dirty and (last_zone_publish is None
                            or timestamp - last_zone_publish >= zone_interval_ns)):
                        self.static_pub.publish(self.zone_markers(timestamp))
                        last_zone_publish = timestamp
                        zones_dirty = False
                    if (last_occupancy_publish is None
                            or timestamp - last_occupancy_publish >= occupancy_interval_ns):
                        self.occupancy_pub.publish(self.occupancy_markers(timestamp))
                        last_occupancy_publish = timestamp
                    continue
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
                draw_zones(frame, self.active_zones, map_to_optical, utm_to_map, camera_matrix,
                           distortion, vertical_offset, 0.25, visible_ranks_out=ranks)
                ortho_to_optical = compose(map_to_ortho, map_to_optical)
                map_to_base = chain_transform(
                    tf_data, [self.config["frames"]["map"], self.config["frames"]["base"]], timestamp,
                    float(self.config["synchronization"]["max_tf_gap_s"]),
                )
                tf_uav = transform_points(map_to_ortho, map_to_base.translation[None, :])[0]
                uav = interpolate_gps(gps, timestamp, vertical_offset, gps_times)
                geographic_error = float(np.linalg.norm(uav[:2] - tf_uav[:2]))
                if not trajectory or np.linalg.norm(uav - trajectory[-1]) > 0.1:
                    trajectory.append(uav)
                footprint, _ = camera_footprint(
                    ortho_to_optical, camera_matrix, distortion, frame.shape[1], frame.shape[0], ground_z,
                    terrain_height=self.terrain.height,
                )
                frame_number += 1
                relative = (timestamp - first_stamp) / 1e9
                text = f"Frame {frame_number:06d} | t={relative:.3f}s | ROS {timestamp} ns"
                status = (f"LZ active {len(self.active_zones)} | blocked {len(self.blocked_zones)} | "
                          f"LiDAR {lidar_frame_number} | GNSS/TF {geographic_error:.2f} m")
                if self.validated_replay is not None:
                    status += (f" | corrected {max(0, pose_index + 1)}/{len(self.validated_replay.poses)} "
                               f"dt {pose_time_error * 1e3:.1f}ms")
                cv2.rectangle(frame, (0, 0), (frame.shape[1], 68), (0, 0, 0), -1)
                cv2.putText(frame, text, (14, 27), cv2.FONT_HERSHEY_SIMPLEX,
                            0.62, (255, 255, 255), 2, cv2.LINE_AA)
                cv2.putText(frame, status, (14, 55), cv2.FONT_HERSHEY_SIMPLEX,
                            0.62, (80, 220, 255), 2, cv2.LINE_AA)
                zones_by_rank = {int(zone["rank"]): zone for zone in self.active_zones}
                replay_markers = self.dynamic_markers(
                    timestamp, [zones_by_rank[rank] for rank in ranks if rank in zones_by_rank], trajectory,
                    uav, ortho_to_optical.translation, footprint, frame_number, tf_uav,
                )
                self.publish_image(frame, timestamp)
                self.dynamic_pub.publish(replay_markers)
                last_publish = timestamp
                if frame_number == 1 or frame_number % 120 == 0:
                    self.get_logger().info(
                        f"Published frame {frame_number} at bag t={relative:.2f}s with {len(ranks)} visible LZs; "
                        f"{len(self.active_zones)} active, {len(self.blocked_zones)} blocked"
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
    parser.add_argument("--ground-pcd", type=Path, default=root / "pcds/696_5328_cropped_ground_only.pcd")
    parser.add_argument("--zones", type=Path, default=root / "pcds/region_growing_landing_zones_max10000_radius2_5_to3_risk_ranked.csv")
    parser.add_argument("--config", type=Path, default=root / "scripts/config/lz_camera_projection.yaml")
    parser.add_argument("--max-points", type=int, default=250000)
    parser.add_argument("--fps", type=float, default=12.0)
    parser.add_argument("--rate", type=float, default=1.0)
    parser.add_argument("--image-width", type=int, default=640)
    parser.add_argument("--optimized-poses", type=Path,
                        default=root / "pcds/bag_only_strip_optimized_poses.npz")
    parser.add_argument("--mapping-config", type=Path,
                        default=root / "scripts/bag_lio_mapping/kiss_icp_config.yaml")
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
