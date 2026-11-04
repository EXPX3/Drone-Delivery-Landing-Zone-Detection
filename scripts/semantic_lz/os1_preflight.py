#!/usr/bin/env python3
"""Validate live OS1, camera, GPS, and TF inputs before enabling LZ processing."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image, NavSatFix, PointCloud2
from tf2_ros import Buffer, TransformListener


class Os1Preflight(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("ddlzd_os1_preflight")
        self.args = args
        self.cloud_count = self.image_count = self.gps_count = 0
        self.cloud_stamps: list[int] = []
        self.cloud_frame = ""
        self.cloud_fields: set[str] = set()
        self.cloud_points = 0
        self.gps_finite = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(
            PointCloud2, args.points_topic, self.on_cloud, qos_profile_sensor_data,
        )
        if args.camera_mode != "off":
            self.create_subscription(
                Image, args.image_topic, self.on_image, qos_profile_sensor_data,
            )
        self.create_subscription(NavSatFix, args.gps_topic, self.on_gps, qos_profile_sensor_data)

    def on_cloud(self, message: PointCloud2) -> None:
        self.cloud_count += 1
        self.cloud_frame = message.header.frame_id.lstrip("/")
        self.cloud_fields = {field.name for field in message.fields}
        self.cloud_points = max(self.cloud_points, int(message.width) * int(message.height))
        self.cloud_stamps.append(
            int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)
        )

    def on_image(self, _message: Image) -> None:
        self.image_count += 1

    def on_gps(self, message: NavSatFix) -> None:
        self.gps_count += 1
        self.gps_finite = bool(np.isfinite([
            message.latitude, message.longitude, message.altitude,
        ]).all())

    def report(self) -> dict:
        duration = max(self.args.duration, 1e-6)
        required_fields = {"x", "y", "z"}
        time_fields = self.cloud_fields.intersection({"time", "t", "timestamp"})
        tf_ready = bool(
            self.cloud_frame
            and self.tf_buffer.can_transform(
                self.args.base_frame, self.cloud_frame, Time(),
                timeout=Duration(seconds=0.25),
            )
        )
        monotonic_stamps = len(self.cloud_stamps) > 1 and bool(
            np.all(np.diff(np.asarray(self.cloud_stamps, dtype=np.int64)) > 0)
        )
        checks = {
            "point_cloud_received": self.cloud_count > 0,
            "xyz_fields": required_fields.issubset(self.cloud_fields),
            "relative_time_field": bool(time_fields),
            "nonempty_frame_id": bool(self.cloud_frame),
            "base_to_lidar_tf": tf_ready,
            "monotonic_cloud_timestamps": monotonic_stamps,
            "finite_gps_received": self.gps_count > 0 and self.gps_finite,
        }
        camera_observed = self.image_count > 0
        checks["camera_images_received"] = camera_observed
        required_checks = [
            "point_cloud_received", "xyz_fields", "relative_time_field",
            "nonempty_frame_id", "base_to_lidar_tf",
            "monotonic_cloud_timestamps", "finite_gps_received",
        ]
        if self.args.camera_mode == "required":
            required_checks.append("camera_images_received")
        return {
            "tag": "os1",
            "ready": all(checks[name] for name in required_checks),
            "duration_s": self.args.duration,
            "checks": checks,
            "required_checks": required_checks,
            "observations": {
                "cloud_topic": self.args.points_topic,
                "cloud_frame": self.cloud_frame,
                "cloud_fields": sorted(self.cloud_fields),
                "maximum_points_per_cloud": self.cloud_points,
                "cloud_messages": self.cloud_count,
                "cloud_rate_hz": self.cloud_count / duration,
                "image_messages": self.image_count,
                "camera_mode": self.args.camera_mode,
                "camera_available": camera_observed,
                "gps_messages": self.gps_count,
                "base_frame": self.args.base_frame,
            },
        }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--points-topic", default="/ouster/points")
    parser.add_argument("--image-topic", default="/GremsyVIO/image/cpu")
    parser.add_argument(
        "--camera-mode", choices=("auto", "required", "off"), default="auto",
        help="auto reports camera availability; required makes it a readiness requirement",
    )
    parser.add_argument("--gps-topic", default="/mavros/global_position/global")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--duration", type=float, default=15.0)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    rclpy.init()
    node = Os1Preflight(args)
    deadline = time.monotonic() + args.duration
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    result = node.report()
    rendered = json.dumps(result, indent=2)
    print(rendered)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")
    node.destroy_node()
    rclpy.shutdown()
    return 0 if result["ready"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
