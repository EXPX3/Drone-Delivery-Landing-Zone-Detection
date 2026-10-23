#!/usr/bin/env python3
"""Publish mapping-audit PCD layers for interactive RViz inspection."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


def read_ascii_pcd(path: Path) -> tuple[list[str], np.ndarray]:
    fields = None
    with path.open("r", encoding="ascii") as handle:
        while True:
            line = handle.readline()
            if not line:
                raise ValueError(f"PCD has no DATA header: {path}")
            tokens = line.strip().split()
            if tokens and tokens[0].upper() == "FIELDS":
                fields = tokens[1:]
            if line.strip().lower() == "data ascii":
                break
        values = np.loadtxt(handle, dtype=np.float64, ndmin=2)
    if fields is None or not {"x", "y", "z"}.issubset(fields):
        raise ValueError(f"PCD does not contain XYZ fields: {path}")
    return fields, values


def cloud_message(fields: list[str], values: np.ndarray, origin: np.ndarray) -> PointCloud2:
    indices = [fields.index(name) for name in ("x", "y", "z")]
    xyz = values[:, indices] - origin
    if "count" in fields:
        intensity = values[:, fields.index("count")]
    else:
        intensity = np.ones(len(values))
    packed = np.ascontiguousarray(np.column_stack((xyz, intensity)), dtype=np.float32)
    message = PointCloud2()
    message.header = Header(frame_id="lz_world")
    message.height = 1
    message.width = len(packed)
    message.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    message.is_bigendian = False
    message.point_step = 16
    message.row_step = message.point_step * message.width
    message.is_dense = True
    message.data = packed.tobytes()
    return message


class AuditPublisher(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("ddlzd_mapping_audit_viewer")
        reference_fields, reference = read_ascii_pcd(args.reference)
        ground_fields, ground = read_ascii_pcd(args.ground)
        change_fields, changes = read_ascii_pcd(args.changes)
        reference_xyz = reference[:, [reference_fields.index(name) for name in ("x", "y", "z")]]
        self.origin = np.median(reference_xyz, axis=0)
        if len(reference) > args.maximum_reference_points:
            index = np.random.default_rng(31).choice(
                len(reference), args.maximum_reference_points, replace=False,
            )
            reference = reference[index]
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        layers = [
            ("/ddlzd/audit/reference_ground", reference_fields, reference),
            ("/ddlzd/audit/corrected_ground", ground_fields, ground),
            ("/ddlzd/audit/scene_changes", change_fields, changes),
        ]
        self._layer_publishers = []
        self._layer_messages = []
        for topic, fields, values in layers:
            publisher = self.create_publisher(PointCloud2, topic, qos)
            message = cloud_message(fields, values, self.origin)
            publisher.publish(message)
            self._layer_publishers.append(publisher)
            self._layer_messages.append(message)
            self.get_logger().info(f"Published {len(values)} points on {topic}")
        self.timer = self.create_timer(2.0, self.republish)
        self.get_logger().info(
            f"Local origin: {self.origin[0]:.3f}, {self.origin[1]:.3f}, {self.origin[2]:.3f}"
        )

    def republish(self) -> None:
        for publisher, message in zip(self._layer_publishers, self._layer_messages):
            message.header.stamp = self.get_clock().now().to_msg()
            publisher.publish(message)


def main() -> int:
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference", type=Path,
                        default=root / "pcds/696_5328_cropped_ground_only.pcd")
    parser.add_argument("--ground", type=Path,
                        default=root / "pcds/lidar_mapping_accuracy/update_only_ground_map.pcd")
    parser.add_argument("--changes", type=Path,
                        default=root / "pcds/lidar_mapping_accuracy/change_only_obstacles.pcd")
    parser.add_argument("--maximum-reference-points", type=int, default=250000)
    args = parser.parse_args()
    rclpy.init()
    node = AuditPublisher(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
