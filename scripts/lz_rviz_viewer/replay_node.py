#!/usr/bin/env python3
"""Fast native ROS 2/RViz replay of the point cloud, camera, frustum, and LZs."""

from __future__ import annotations

import argparse
from collections import Counter
import json
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
from std_msgs.msg import ColorRGBA, Header, String
from visualization_msgs.msg import Marker, MarkerArray

SCRIPTS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPTS))

from lz_bag_replay_viewer.replay_server import (  # noqa: E402
    camera_footprint, displayed_zone_ring, orthometric_from_map,
)
from lz_camera_projection import (  # noqa: E402
    compose, load_landing_zones, project_optical_points, scaled_camera_matrix,
    transform_from_matrix, transform_points,
)
from lz_rviz_viewer.dynamic_occupancy import (  # noqa: E402
    apply_centered_transform, DynamicOccupancyMap, interpolate_gps, pointcloud2_xyz,
    TerrainHeightModel,
)
from lz_rviz_viewer.validated_pose_replay import ValidatedPoseReplay  # noqa: E402
from project_lz_into_bag_camera import (  # noqa: E402
    chain_transform, config_transform, decode_image, draw_zones, fit_utm_to_map,
    open_reader, read_pose_and_gps, require_ros, stamp_ns,
)
from semantic_lz.semantic_fusion import (  # noqa: E402
    CLASS_COLORS_RGB, STATIC_CLASSES, OpenVocabularySegmenter, SemanticObjectTracker,
    VisionProposalTracker, colorize_frame, compact_supported_clusters,
    filter_semantic_result, labels_at_pixels,
)
from semantic_lz.ground_reference import (  # noqa: E402
    GROUND_REFERENCE_MODES, estimate_detection_ground,
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


def colored_point_cloud_message(points: np.ndarray, colors: np.ndarray,
                                origin: np.ndarray) -> PointCloud2:
    local = np.ascontiguousarray(points - origin, dtype=np.float32)
    packed = ((colors[:, 0].astype(np.uint32) << 16)
              | (colors[:, 1].astype(np.uint32) << 8)
              | colors[:, 2].astype(np.uint32))
    records = np.empty(len(local), dtype=[("x", "<f4"), ("y", "<f4"),
                                           ("z", "<f4"), ("rgb", "<u4")])
    records["x"], records["y"], records["z"], records["rgb"] = (
        local[:, 0], local[:, 1], local[:, 2], packed,
    )
    message = PointCloud2(header=Header(frame_id=FRAME_ID), height=1, width=len(records))
    message.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1),
    ]
    message.point_step = 16
    message.row_step = 16 * len(records)
    message.is_dense = True
    message.data = records.tobytes()
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


def point_distribution(points: np.ndarray) -> dict:
    """Compact robust statistics for comparing semantic and physical support."""
    values = np.asarray(points, dtype=np.float64)
    if not len(values):
        return {"count": 0}
    center = np.median(values, axis=0)
    offsets = values[:, :2] - center[:2]
    radial = np.linalg.norm(offsets, axis=1)
    return {
        "count": int(len(values)),
        "median_xyz": center.tolist(),
        "std_xyz": np.std(values, axis=0).tolist(),
        "radial_p95_m": float(np.percentile(radial, 95)),
        "xy_min": np.min(values[:, :2], axis=0).tolist(),
        "xy_max": np.max(values[:, :2], axis=0).tolist(),
    }


class LandingZoneRvizReplay(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("landing_zone_rviz_replay")
        self.args = args
        self.metrics = {
            "detections": Counter(), "projected_points": Counter(),
            "above_ground_points": Counter(), "guided_points": Counter(),
            "height_residuals_m": {}, "semantic_observations": [],
        }
        self.stop = threading.Event()
        with args.config.open(encoding="utf-8") as handle:
            self.config = yaml.safe_load(handle)
        dynamic_overrides = self.config["dynamic_occupancy"]
        if args.point_stride is not None:
            dynamic_overrides["point_stride"] = args.point_stride
        if args.rolling_radius is not None:
            dynamic_overrides["rolling_horizontal_radius_m"] = args.rolling_radius
        if args.confirmation_hits is not None:
            dynamic_overrides["confirmation_hits"] = args.confirmation_hits
        if args.semantic_min_height is not None:
            self.config["semantic_segmentation"]["minimum_height_above_ground_m"] = (
                args.semantic_min_height
            )
        if args.semantic_ground_reference is not None:
            self.config["semantic_segmentation"]["ground_reference_mode"] = (
                args.semantic_ground_reference
            )
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
            confirmation_window_scans=int(dynamic["confirmation_window_scans"]),
            clustering=dynamic.get("point_clustering"),
            hazard_clear_after_scans=int(dynamic.get("hazard_clear_after_scans", 15)),
            hazard_clearing_radius_m=float(dynamic.get("hazard_clearing_radius_m", 20.0)),
            hazard_clear_ray_margin_m=float(dynamic.get("hazard_clear_ray_margin_m", 0.30)),
            hazard_clear_min_angle_deg=float(dynamic.get("hazard_clear_min_angle_deg", 0.15)),
            clear_ground_tolerance_m=float(dynamic.get("clear_ground_tolerance_m", 0.12)),
            clear_grid_resolution_m=float(dynamic.get("clear_grid_resolution_m", 0.50)),
            clear_minimum_ground_points=int(dynamic.get("clear_minimum_ground_points", 20)),
            clear_minimum_coverage=float(dynamic.get("clear_minimum_coverage", 0.25)),
            clear_minimum_hazard_age_scans=int(
                dynamic.get("clear_minimum_hazard_age_scans", 0)
            ),
            local_ground=dynamic.get("local_ground_plane"),
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
        self.occupancy_cloud_pub = self.create_publisher(
            PointCloud2, "/ddlzd/live_occupancy_points", durable,
        )
        self.image_pub = self.create_publisher(Image, "/ddlzd/camera_overlay", live)
        self.semantic_pub = self.create_publisher(
            PointCloud2, "/ddlzd/semantic_obstacles", durable,
        )
        self.semantic_tracks_pub = self.create_publisher(
            String, "/ddlzd/semantic_object_uncertainty", live,
        )
        semantic = self.config.get("semantic_segmentation", {})
        self.segmenter = None
        if bool(semantic.get("enabled", False)):
            self.segmenter = OpenVocabularySegmenter(
                self.args.semantic_weights, list(semantic["prompts"]),
                confidence=float(semantic["confidence"]),
                image_size=int(semantic["image_size"]), device=str(semantic["device"]),
            )
        verification = semantic.get("vision_guided_lidar", {})
        self.vision_proposal_tracker = VisionProposalTracker(
            association_radius_m=float(
                verification.get("vision_proposal_association_radius_m", 2.0)
            ),
            retention_s={
                str(name): (None if value is None else float(value))
                for name, value in verification.get(
                    "vision_proposal_retention_s",
                    {"person": 2.0, "cargo_box": 3.0, "vehicle": 3.0},
                ).items()
            },
            merge_radius_m=float(
                verification.get("vision_proposal_merge_radius_m", 2.5)
            ),
            mahalanobis_gate=float(
                verification.get("vision_proposal_mahalanobis_gate", 9.21)
            ),
            history_size=int(verification.get("history_size", 60)),
            systematic_xy_std_m=float(
                verification.get("vision_proposal_systematic_xy_std_m", 0.75)
            ),
            confirmation_observations={
                str(name): int(value) for name, value in verification.get(
                    "vision_proposal_confirmation_observations",
                    {"person": 1, "cargo_box": 3, "vehicle": 2},
                ).items()
            },
            tentative_retention_s=float(
                verification.get("vision_proposal_tentative_retention_s", 10.0)
            ),
        )
        self.guided_lidar_tracker = SemanticObjectTracker(
            cluster_radius_m=float(verification.get("cluster_radius_m", 0.75)),
            association_radius_m=float(verification.get("association_radius_m", 1.2)),
            confirmation_frames=int(verification.get("confirmation_frames", 2)),
            maximum_age_frames=int(verification.get("maximum_age_frames", 90)),
            history_size=int(verification.get("history_size", 60)),
            duplicate_merge_radius_m=float(
                verification.get("duplicate_merge_radius_m", 1.5)
            ),
            confirmed_duplicate_merge_radius_m=float(
                verification.get("confirmed_duplicate_merge_radius_m", 2.5)
            ),
            static_voxel_size_m=float(verification.get("static_voxel_size_m", 0.10)),
        )
        self.cloud_pub.publish(point_cloud_message(cloud, self.origin))
        self.static_pub.publish(self.zone_markers(0))
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

    def occupancy_markers(self, timestamp_ns: int, include_baseline: bool = False,
                          center: np.ndarray | None = None,
                          radius_m: float | None = None) -> MarkerArray:
        limit = int(self.config["dynamic_occupancy"]["max_visualized_voxels"])
        layers = self.occupancy.occupancy_layers(
            include_baseline=include_baseline, center=center,
            horizontal_radius_m=radius_m,
        )
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
                        frame_number: int, gnss_uav: np.ndarray,
                        vision_proposals: list[dict] | None = None) -> MarkerArray:
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
        for value in (uav, gnss_uav):
            local = value - self.origin
            registration.points.append(Point(x=float(local[0]), y=float(local[1]), z=float(local[2])))
        semantic_markers: list[Marker] = []
        for proposal_index, proposal in enumerate(vision_proposals or []):
            center = np.asarray(proposal["center"], dtype=np.float64)
            color_rgb = np.asarray(CLASS_COLORS_RGB[proposal["class_name"]]) / 255.0
            proposal_marker = marker(
                8000 + 3 * proposal_index, "vision_only_proposals", Marker.SPHERE,
                (*map(float, color_rgb), 0.72), 0.36, timestamp_ns,
            )
            local = center - self.origin
            proposal_marker.pose.position = Point(
                x=float(local[0]), y=float(local[1]), z=float(local[2] + 0.18),
            )
            semantic_markers.append(proposal_marker)
            covariance = np.asarray(proposal["covariance_xy"], dtype=np.float64)
            eigenvalues, eigenvectors = np.linalg.eigh(covariance)
            angles = np.linspace(0.0, 2.0 * np.pi, 49)
            axes = 2.0 * np.sqrt(np.maximum(eigenvalues, 0.04))
            ellipse = marker(
                8001 + 3 * proposal_index, "vision_only_covariance", Marker.LINE_STRIP,
                (*map(float, color_rgb), 0.55), 0.06, timestamp_ns,
            )
            for angle in angles:
                offset = eigenvectors @ (axes * [np.cos(angle), np.sin(angle)])
                ellipse.points.append(Point(
                    x=float(local[0] + offset[0]), y=float(local[1] + offset[1]),
                    z=float(local[2] + 0.20),
                ))
            semantic_markers.append(ellipse)
            proposal_label = marker(
                8002 + 3 * proposal_index, "vision_only_labels", Marker.TEXT_VIEW_FACING,
                (*map(float, color_rgb), 1.0), 0.30, timestamp_ns,
            )
            proposal_label.pose.position = Point(
                x=float(local[0]), y=float(local[1]), z=float(local[2] + 0.70),
            )
            proposal_label.text = (
                f"{proposal['class_name']} | vision provisional "
                f"#{proposal['proposal_id']} | conf {proposal['confidence']:.2f} "
                f"| {'confirmed' if proposal['confirmed'] else 'tentative'} "
                f"| obs {proposal['observations']} | last seen "
                f"{max(0.0, (timestamp_ns - proposal['last_seen_ns']) / 1e9):.1f}s"
            )
            semantic_markers.append(proposal_label)
        for track in self.guided_lidar_tracker.tracks:
            centers = np.asarray(track.centers, dtype=np.float64)
            covariance = (np.cov(centers[:, :2], rowvar=False)
                          if len(centers) > 1 else np.eye(2) * 0.01)
            eigenvalues, eigenvectors = np.linalg.eigh(covariance)
            order = np.argsort(eigenvalues)[::-1]
            eigenvalues, eigenvectors = eigenvalues[order], eigenvectors[:, order]
            center = np.median(centers, axis=0)
            color_rgb = self.guided_lidar_tracker.track_color(track) / 255.0
            rgb = (float(color_rgb[0]), float(color_rgb[1]), float(color_rgb[2]))
            centroid_marker = marker(
                10000 + 10 * track.track_id, "semantic_track_centroids",
                Marker.SPHERE, (*rgb, 1.0), 0.48, timestamp_ns,
            )
            centroid_local = center - self.origin
            centroid_marker.pose.position = Point(
                x=float(centroid_local[0]), y=float(centroid_local[1]),
                z=float(centroid_local[2] + 0.25),
            )
            semantic_markers.append(centroid_marker)
            angles = np.linspace(0.0, 2.0 * np.pi, 49)
            sigma_axes = np.maximum(np.sqrt(np.maximum(eigenvalues, 0.0)), 0.05)
            local = center - self.origin
            # Oriented 1/2/3-sigma projections of the empirical XY covariance.
            for sigma_index, alpha in ((1, 1.0), (2, 0.72), (3, 0.46)):
                contour = marker(
                    10000 + 10 * track.track_id + sigma_index,
                    f"semantic_covariance_{sigma_index}sigma", Marker.LINE_STRIP,
                    (*rgb, alpha), 0.05 + 0.02 * (4 - sigma_index), timestamp_ns,
                )
                axes = sigma_index * sigma_axes
                offsets = (eigenvectors @ np.vstack((
                    axes[0] * np.cos(angles), axes[1] * np.sin(angles),
                ))).T
                for offset in offsets:
                    contour.points.append(Point(x=float(local[0] + offset[0]),
                                                y=float(local[1] + offset[1]),
                                                z=float(local[2] + 0.30)))
                semantic_markers.append(contour)

            # Normalized bivariate-Gaussian wireframe. Its footprint is the same
            # covariance as the contours; Z shows relative density, not altitude.
            bell = marker(10000 + 10 * track.track_id + 4, "semantic_gaussian_surface",
                          Marker.LINE_LIST, (*rgb, 0.72), 0.035, timestamp_ns)
            bell_radii = np.linspace(0.0, 3.0, 7)
            bell_angles = np.linspace(0.0, 2.0 * np.pi, 33)

            def bell_point(radius: float, angle: float) -> Point:
                offset = eigenvectors @ np.array([
                    radius * sigma_axes[0] * np.cos(angle),
                    radius * sigma_axes[1] * np.sin(angle),
                ])
                height = 0.30 + 0.90 * np.exp(-0.5 * radius * radius)
                return Point(x=float(local[0] + offset[0]),
                             y=float(local[1] + offset[1]),
                             z=float(local[2] + height))

            for radius in bell_radii[1:]:
                for angle_index in range(len(bell_angles) - 1):
                    bell.points.extend((bell_point(radius, bell_angles[angle_index]),
                                        bell_point(radius, bell_angles[angle_index + 1])))
            for angle in bell_angles[::4]:
                for radius_index in range(len(bell_radii) - 1):
                    bell.points.extend((bell_point(bell_radii[radius_index], angle),
                                        bell_point(bell_radii[radius_index + 1], angle)))
            semantic_markers.append(bell)

            text_marker = marker(10000 + 10 * track.track_id + 5, "semantic_track_labels",
                                 Marker.TEXT_VIEW_FACING, (*rgb, 1.0), 0.38, timestamp_ns)
            local = center - self.origin
            text_marker.pose.position = Point(x=float(local[0]), y=float(local[1]),
                                              z=float(local[2] + 1.35))
            radial = np.linalg.norm(centers[:, :2] - center[:2], axis=1)
            text_marker.text = (
                f"{track.class_name} #{track.display_id or track.track_id} "
                f"| {'confirmed' if track.observations >= self.guided_lidar_tracker.confirmation_frames else 'provisional'} "
                f"| P95 {np.percentile(radial, 95):.2f} m\n"
                f"Cov XY [m^2] [{covariance[0, 0]:.3f} {covariance[0, 1]:+.3f}; "
                f"{covariance[1, 0]:+.3f} {covariance[1, 1]:.3f}]"
            )
            semantic_markers.append(text_marker)
        return MarkerArray(markers=[
            clear, pink, trail, aircraft, outline, rays, label, registration,
            *semantic_markers,
        ])

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
            latest_lidar_points = np.empty((0, 3), dtype=np.float64)
            latest_lidar_stamp = 0
            pose_index = -1
            pose_time_error = 0.0
            interval_ns = int(1e9 / self.args.fps)
            occupancy_interval_ns = int(1e9 / float(self.config["dynamic_occupancy"]["publish_hz"]))
            zone_interval_ns = int(1e9 / float(self.config["dynamic_occupancy"]["zone_publish_hz"]))
            max_lidar_range = float(self.config["dynamic_occupancy"]["maximum_lidar_range_m"])
            rolling_radius = float(
                self.config["dynamic_occupancy"]["rolling_horizontal_radius_m"]
            )
            lidar_stride = int(self.config["dynamic_occupancy"]["point_stride"])
            map_stride = int(self.config["dynamic_occupancy"].get("map_point_stride", 1))
            corrected_alignment_ready = False
            lidar_timings: dict[str, list[float]] = {
                "transform": [], "occupancy": [], "zone_publish": [],
                "map_publish": [], "total": [],
            }
            while rclpy.ok() and not self.stop.is_set() and reader.has_next():
                topic, raw, bag_time = reader.read_next()
                if topic == lidar_topic:
                    scan_started = time.perf_counter()
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
                            lidar_origin = self.validated_replay.sensor_origin(pose_index)
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
                            lidar_origin = apply_centered_transform(
                                transform_points(map_to_ortho, map_to_lidar.translation[None, :]),
                                self.lidar_correction.matrix, self.lidar_registration_origin,
                            )[0]
                            pose_index, pose_time_error = lidar_frame_number, 0.0
                    except LookupError as error:
                        self.get_logger().warning(str(error))
                        continue
                    transformed_at = time.perf_counter()
                    latest_lidar_points = ortho_points
                    latest_lidar_stamp = timestamp
                    horizontal_distance = np.linalg.norm(
                        ortho_points[:, :2] - lidar_origin[:2], axis=1,
                    )
                    hazard_points = ortho_points[horizontal_distance <= rolling_radius]
                    changed = self.occupancy.update(
                        ortho_points[::map_stride], lidar_origin, hazard_points=hazard_points,
                    )
                    occupied_at = time.perf_counter()
                    lidar_frame_number += 1
                    if changed:
                        self.active_zones, self.blocked_zones = self.occupancy.classified_zones()
                        zones_dirty = True
                    if (zones_dirty and (last_zone_publish is None
                            or timestamp - last_zone_publish >= zone_interval_ns)):
                        if self.args.metrics_json is None:
                            self.static_pub.publish(self.zone_markers(timestamp))
                        last_zone_publish = timestamp
                        zones_dirty = False
                    zone_published_at = time.perf_counter()
                    if (last_occupancy_publish is None
                            or timestamp - last_occupancy_publish >= occupancy_interval_ns):
                        if self.args.metrics_json is None:
                            live_map = self.occupancy.occupied_centers(include_baseline=False)
                            map_limit = int(self.config["dynamic_occupancy"].get(
                                "max_visualized_map_points", 150000,
                            ))
                            if len(live_map) > map_limit:
                                indices = np.linspace(
                                    0, len(live_map) - 1, map_limit, dtype=np.int64,
                                )
                                live_map = live_map[indices]
                            cloud_message = point_cloud_message(live_map, self.origin)
                            ros_stamp(cloud_message, timestamp)
                            self.occupancy_cloud_pub.publish(cloud_message)
                        last_occupancy_publish = timestamp
                    scan_finished = time.perf_counter()
                    for name, value in (
                        ("transform", transformed_at - scan_started),
                        ("occupancy", occupied_at - transformed_at),
                        ("zone_publish", zone_published_at - occupied_at),
                        ("map_publish", scan_finished - zone_published_at),
                        ("total", scan_finished - scan_started),
                    ):
                        lidar_timings[name].append(value * 1000.0)
                    if lidar_frame_number % 100 == 0:
                        report = " | ".join(
                            f"{name} {np.mean(values[-100:]):.1f}/{np.percentile(values[-100:], 95):.1f}ms"
                            for name, values in lidar_timings.items()
                        )
                        self.get_logger().info(f"LiDAR timing mean/P95: {report}")
                    continue
                if topic != image_topic:
                    continue
                message = deserialize_message(raw, image_type)
                timestamp = stamp_ns(message, bag_time)
                if first_stamp is None:
                    first_stamp = timestamp
                if (self.args.max_bag_seconds is not None
                        and timestamp - first_stamp > self.args.max_bag_seconds * 1e9):
                    break
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
                corrected_body_pose = None
                if self.validated_replay is not None and corrected_alignment_ready:
                    map_to_base = chain_transform(
                        tf_data, [self.config["frames"]["map"], self.config["frames"]["base"]],
                        timestamp, float(self.config["synchronization"]["max_tf_gap_s"]),
                    )
                    body_to_mount = np.linalg.inv(map_to_base.matrix) @ map_to_mount.matrix
                    corrected_body_pose = self.validated_replay.world_body_pose(timestamp)
                    world_to_mount = corrected_body_pose @ body_to_mount
                    ortho_to_optical = compose(
                        transform_from_matrix(world_to_mount),
                        config_transform(self.config["camera"]["mount_to_optical"]),
                    )
                    map_to_optical = compose(map_to_ortho.inverse(), ortho_to_optical)
                semantic_result = None
                vision_proposals: list[dict] = []
                semantic = self.config.get("semantic_segmentation", {})
                inference_stride = int(semantic.get("inference_stride", 1))
                if self.segmenter is not None and frame_number % inference_stride == 0:
                    semantic_result = self.segmenter.predict(frame)
                    self.metrics["detections"].update(semantic_result.class_names)
                ranks: list[int] = []
                draw_zones(frame, self.active_zones, map_to_optical, utm_to_map, camera_matrix,
                           distortion, vertical_offset, 0.25, visible_ranks_out=ranks)
                ortho_to_optical = compose(map_to_ortho, map_to_optical)
                semantic_counts: dict[str, int] = {}
                if (semantic_result is not None and len(latest_lidar_points)
                        and abs(timestamp - latest_lidar_stamp) / 1e9
                        <= float(semantic.get("maximum_lidar_age_s", 0.15))):
                    optical_points = transform_points(
                        ortho_to_optical.inverse(), latest_lidar_points,
                    )
                    pixels, in_front = project_optical_points(
                        optical_points, camera_matrix, distortion,
                    )
                    labels = labels_at_pixels(semantic_result, pixels, in_front)
                    projected_names = np.asarray(semantic_result.class_names, dtype=object)[
                        labels[labels > 0] - 1
                    ]
                    self.metrics["projected_points"].update(map(str, projected_names))
                    selected = labels > 0
                    if np.any(selected):
                        semantic_points = latest_lidar_points[selected]
                        names = np.asarray(semantic_result.class_names, dtype=object)[labels[selected] - 1]
                        selected_detection = labels[selected] - 1
                        minimum_height = max(
                            0.0, float(semantic.get("minimum_height_above_ground_m", 0.15))
                        )
                        maximum_height = float(semantic.get("maximum_height_above_ground_m", 3.0))
                        ground_mode = str(semantic.get(
                            "ground_reference_mode", "bayern_ground_reference",
                        ))
                        ground = np.full(len(semantic_points), np.nan, dtype=np.float64)
                        ground_diagnostics: dict[int, dict] = {}
                        for detection_index in range(len(semantic_result.class_names)):
                            detection_selected = selected_detection == detection_index
                            estimate = estimate_detection_ground(
                                ground_mode, semantic_points[detection_selected],
                                latest_lidar_points, self.terrain,
                                semantic.get("local_detection_ground", {}),
                            )
                            ground[detection_selected] = estimate.ground_z
                            ground_diagnostics[detection_index] = estimate.diagnostics
                        for class_name in set(names):
                            values = [
                                float(point[2] - value)
                                for point, value, name in zip(semantic_points, ground, names)
                                if name == class_name and np.isfinite(value)
                            ]
                            self.metrics["height_residuals_m"].setdefault(
                                str(class_name), []
                            ).extend(values)
                        height_ok = np.asarray([
                            np.isfinite(value) and minimum_height <= point[2] - value <= maximum_height
                            for point, value in zip(semantic_points, ground)
                        ])
                        if self.args.metrics_json is not None:
                            yaw_deg = None
                            if corrected_body_pose is not None:
                                yaw_deg = float(np.degrees(np.arctan2(
                                    corrected_body_pose[1, 0], corrected_body_pose[0, 0],
                                )))
                            for detection_index, (class_name, confidence, mask) in enumerate(zip(
                                    semantic_result.class_names,
                                    semantic_result.confidences,
                                    semantic_result.masks)):
                                detection_selected = selected_detection == detection_index
                                painted = semantic_points[detection_selected]
                                verified = semantic_points[detection_selected & height_ok]
                                raw_local = np.empty((0, 3), dtype=np.float64)
                                if len(verified):
                                    center = np.median(verified[:, :2], axis=0)
                                    distance = np.linalg.norm(
                                        latest_lidar_points[:, :2] - center, axis=1,
                                    )
                                    local_candidates = latest_lidar_points[distance <= 1.5]
                                    local_estimate = estimate_detection_ground(
                                        ground_mode, local_candidates, latest_lidar_points,
                                        self.terrain, semantic.get("local_detection_ground", {}),
                                    )
                                    local_ground = local_estimate.ground_z
                                    local_ok = np.asarray([
                                        np.isfinite(value) and minimum_height
                                        <= point[2] - value <= maximum_height
                                        for point, value in zip(local_candidates, local_ground)
                                    ])
                                    raw_local = local_candidates[local_ok]
                                self.metrics["semantic_observations"].append({
                                    "timestamp_ns": int(timestamp),
                                    "relative_time_s": float((timestamp - first_stamp) / 1e9),
                                    "image_frame": int(frame_number + 1),
                                    "class_name": str(class_name),
                                    "confidence": float(confidence),
                                    "mask_fraction": float(np.count_nonzero(mask) / mask.size),
                                    "lidar_age_ms": float((timestamp - latest_lidar_stamp) / 1e6),
                                    "uav_yaw_deg": yaw_deg,
                                    "painted": point_distribution(painted),
                                    "height_verified": point_distribution(verified),
                                    "raw_local_elevated": point_distribution(raw_local),
                                    "ground_reference": ground_diagnostics[detection_index],
                                })
                        semantic_points = semantic_points[height_ok]
                        names = names[height_ok]
                        self.metrics["above_ground_points"].update(map(str, names))
                        verification = semantic.get("vision_guided_lidar", {})
                        thresholds = verification.get("minimum_image_confidence", {})
                        mask_limits = verification.get("maximum_mask_fraction", {})
                        radial_limits = verification.get("maximum_radial_p95_m", {})
                        minimum_by_class = verification.get("minimum_lidar_points_by_class", {})
                        point_detection = selected_detection[height_ok]
                        guided_points: list[np.ndarray] = []
                        guided_names: list[str] = []
                        guided_confidences: list[float] = []
                        for detection_index, (class_name, confidence, mask) in enumerate(zip(
                                semantic_result.class_names,
                                semantic_result.confidences,
                                semantic_result.masks)):
                            class_name = str(class_name)
                            if confidence < float(thresholds.get(class_name, 1.0)):
                                continue
                            if (np.count_nonzero(mask) / mask.size
                                    > float(mask_limits.get(class_name, 1.0))):
                                continue
                            detection_points = semantic_points[point_detection == detection_index]
                            clusters = compact_supported_clusters(
                                detection_points, class_name,
                                float(verification.get("cluster_radius_m", 0.75)),
                                int(minimum_by_class.get(
                                    class_name,
                                    verification.get("minimum_lidar_points_per_frame", 2),
                                )),
                                float(radial_limits.get(class_name, np.inf)),
                            )
                            for cluster in clusters:
                                guided_points.extend(cluster)
                                guided_names.extend([class_name] * len(cluster))
                                guided_confidences.extend([float(confidence)] * len(cluster))
                        if guided_points:
                            self.metrics["guided_points"].update(guided_names)
                            self.guided_lidar_tracker.update(
                                np.asarray(guided_points), np.asarray(guided_names, dtype=object),
                                frame_index=frame_number + 1,
                                confidences=np.asarray(guided_confidences),
                            )
                        else:
                            self.guided_lidar_tracker.expire(frame_number + 1)
                        semantic_counts = {
                            name: int(np.count_nonzero(names == name)) for name in set(names)
                        }
                if semantic_result is not None:
                    verification = semantic.get("vision_guided_lidar", {})
                    thresholds = verification.get("minimum_image_confidence", {})
                    mask_limits = verification.get("maximum_mask_fraction", {})
                    for class_name, confidence, mask in zip(
                            semantic_result.class_names, semantic_result.confidences,
                            semantic_result.masks):
                        class_name = str(class_name)
                        mask_fraction = float(np.count_nonzero(mask) / mask.size)
                        if (confidence < float(thresholds.get(class_name, 1.0))
                                or mask_fraction > float(mask_limits.get(class_name, 1.0))):
                            continue
                        rows, columns = np.nonzero(mask)
                        if not len(rows):
                            continue
                        sample = np.linspace(0, len(rows) - 1, min(500, len(rows)), dtype=np.int64)
                        pixels_to_project = np.column_stack((columns[sample], rows[sample])).astype(np.float64)
                        normalized = cv2.undistortPoints(
                            pixels_to_project.reshape(-1, 1, 2), camera_matrix, distortion,
                        ).reshape(-1, 2)
                        rays = (ortho_to_optical.matrix[:3, :3]
                                @ np.column_stack((normalized, np.ones(len(normalized)))).T).T
                        valid = rays[:, 2] < -1e-6
                        ground_points = []
                        for ray in rays[valid]:
                            scale = (ground_z - ortho_to_optical.translation[2]) / ray[2]
                            for _ in range(4):
                                value = ortho_to_optical.translation + scale * ray
                                terrain_z = self.terrain.height(float(value[0]), float(value[1]))
                                if terrain_z is None:
                                    break
                                scale = (terrain_z - ortho_to_optical.translation[2]) / ray[2]
                            if scale > 0.0:
                                ground_points.append(ortho_to_optical.translation + scale * ray)
                        if not ground_points:
                            continue
                        projected = np.asarray(ground_points)
                        center = np.median(projected, axis=0)
                        covariance = (np.cov(projected[:, :2], rowvar=False)
                                      if len(projected) > 1 else np.eye(2) * 0.25)
                        vision_proposals.append({
                            "class_name": class_name, "confidence": float(confidence),
                            "center": center, "covariance_xy": covariance,
                        })
                    self.vision_proposal_tracker.update(vision_proposals, timestamp)
                    frame = colorize_frame(frame, filter_semantic_result(
                        semantic_result,
                        thresholds, mask_limits,
                    ))
                semantic_points, semantic_colors = self.guided_lidar_tracker.point_cloud()
                active_vision_proposals = self.vision_proposal_tracker.active(timestamp)
                if active_vision_proposals:
                    proposal_points = np.asarray([
                        proposal["center"] for proposal in active_vision_proposals
                    ], dtype=np.float64)
                    proposal_colors = np.asarray([
                        CLASS_COLORS_RGB[proposal["class_name"]]
                        for proposal in active_vision_proposals
                    ], dtype=np.uint8)
                    semantic_points = (np.vstack((semantic_points, proposal_points))
                                       if len(semantic_points) else proposal_points)
                    semantic_colors = (np.vstack((semantic_colors, proposal_colors))
                                       if len(semantic_colors) else proposal_colors)
                current_frame = frame_number + 1
                static_updates = [
                    track.accumulated_points
                    for track in self.guided_lidar_tracker.tracks
                    if (track.class_name in STATIC_CLASSES
                        and track.last_frame == current_frame
                        and track.observations >= self.guided_lidar_tracker.confirmation_frames)
                ]
                dynamic_support = [
                    track.points for track in self.guided_lidar_tracker.tracks
                    if track.class_name not in STATIC_CLASSES
                ]
                semantic_zone_changes = 0
                if static_updates:
                    semantic_zone_changes += self.occupancy.add_confirmed_lidar_obstacles(
                        np.vstack(static_updates), persistent=True,
                    )
                semantic_zone_changes += self.occupancy.set_dynamic_lidar_obstacles(
                    np.vstack(dynamic_support)
                    if dynamic_support else np.empty((0, 3), dtype=np.float64)
                )
                if semantic_zone_changes:
                    self.active_zones, self.blocked_zones = self.occupancy.classified_zones()
                    if self.args.metrics_json is None:
                        self.static_pub.publish(self.zone_markers(timestamp))
                semantic_cloud = colored_point_cloud_message(
                    semantic_points, semantic_colors, self.origin,
                )
                ros_stamp(semantic_cloud, timestamp)
                self.semantic_pub.publish(semantic_cloud)
                self.semantic_tracks_pub.publish(String(data=json.dumps({
                    "timestamp_ns": int(timestamp),
                    "frame": int(frame_number + 1),
                    "tracks": self.guided_lidar_tracker.summaries(),
                })))
                map_to_base = chain_transform(
                    tf_data, [self.config["frames"]["map"], self.config["frames"]["base"]], timestamp,
                    float(self.config["synchronization"]["max_tf_gap_s"]),
                )
                raw_tf_uav = transform_points(map_to_ortho, map_to_base.translation[None, :])[0]
                gnss_uav = interpolate_gps(gps, timestamp, vertical_offset, gps_times)
                uav = (corrected_body_pose[:3, 3].copy()
                       if corrected_body_pose is not None else raw_tf_uav)
                geographic_error = float(np.linalg.norm(gnss_uav[:2] - uav[:2]))
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
                          f"LiDAR {lidar_frame_number} | GNSS/corrected pose {geographic_error:.2f} m")
                if self.segmenter is not None:
                    status += f" | semantic {semantic_counts or 'none'}"
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
                    uav, ortho_to_optical.translation, footprint, frame_number, gnss_uav,
                    active_vision_proposals,
                )
                if self.args.metrics_json is None:
                    self.publish_image(frame, timestamp)
                    self.dynamic_pub.publish(replay_markers)
                last_publish = timestamp
                if frame_number == 1 or frame_number % 120 == 0:
                    self.get_logger().info(
                        f"Published frame {frame_number} at bag t={relative:.2f}s with {len(ranks)} visible LZs; "
                        f"{len(self.active_zones)} active, {len(self.blocked_zones)} blocked"
                    )
            self.get_logger().info("Replay complete")
            if self.args.metrics_json is not None:
                report = {}
                for key, value in self.metrics.items():
                    if isinstance(value, Counter):
                        report[key] = dict(value)
                    elif key == "height_residuals_m":
                        report[key] = {
                            name: {
                                "count": len(values),
                                "minimum": min(values) if values else None,
                                "median": float(np.median(values)) if values else None,
                                "p95": float(np.percentile(values, 95)) if values else None,
                                "maximum": max(values) if values else None,
                            }
                            for name, values in value.items()
                        }
                    else:
                        report[key] = value
                report.update({
                    "semantic_ground_reference_mode": self.config.get(
                        "semantic_segmentation", {}
                    ).get("ground_reference_mode", "bayern_ground_reference"),
                    "guided_lidar_tracks": self.guided_lidar_tracker.summaries(),
                    "active_landing_zones": len(self.active_zones),
                    "blocked_landing_zones": len(self.blocked_zones),
                    "lidar_frames": lidar_frame_number,
                    "image_frames": frame_number,
                })
                self.args.metrics_json.parent.mkdir(parents=True, exist_ok=True)
                self.args.metrics_json.write_text(json.dumps(report, indent=2), encoding="utf-8")
                rclpy.shutdown()
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
    parser.add_argument("--semantic-weights", type=str,
                        default=str(root / "pcds/semantic_experiment/models/yoloe-11s-seg.pt"))
    parser.add_argument("--metrics-json", type=Path)
    parser.add_argument("--point-stride", type=int)
    parser.add_argument("--rolling-radius", type=float)
    parser.add_argument("--confirmation-hits", type=int)
    parser.add_argument("--semantic-min-height", type=float)
    parser.add_argument("--semantic-ground-reference", choices=sorted(GROUND_REFERENCE_MODES))
    parser.add_argument("--max-bag-seconds", type=float)
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
