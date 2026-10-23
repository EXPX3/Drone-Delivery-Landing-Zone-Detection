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


def read_pcd(path: Path) -> tuple[list[str], np.ndarray]:
    fields = sizes = types = counts = None
    point_count = 0
    with path.open("rb") as handle:
        while True:
            line = handle.readline()
            if not line:
                raise ValueError(f"PCD has no DATA header: {path}")
            tokens = line.decode("ascii").strip().split()
            if tokens and tokens[0].upper() == "FIELDS":
                fields = tokens[1:]
            elif tokens and tokens[0].upper() == "SIZE":
                sizes = [int(value) for value in tokens[1:]]
            elif tokens and tokens[0].upper() == "TYPE":
                types = tokens[1:]
            elif tokens and tokens[0].upper() == "COUNT":
                counts = [int(value) for value in tokens[1:]]
            elif tokens and tokens[0].upper() == "POINTS":
                point_count = int(tokens[1])
            if tokens and tokens[0].upper() == "DATA":
                encoding = tokens[1].lower()
                break
        if fields is None or not {"x", "y", "z"}.issubset(fields):
            raise ValueError(f"PCD does not contain XYZ fields: {path}")
        if encoding == "ascii":
            values = np.loadtxt(handle, dtype=np.float64, ndmin=2)
        elif encoding == "binary":
            if sizes is None or types is None:
                raise ValueError(f"Binary PCD has incomplete metadata: {path}")
            counts = counts or [1] * len(fields)
            formats = []
            supported = {("F", 4): "<f4", ("F", 8): "<f8", ("U", 4): "<u4", ("I", 4): "<i4"}
            for size, field_type, count in zip(sizes, types, counts):
                code = supported.get((field_type.upper(), size))
                if code is None or count != 1:
                    raise ValueError(f"Unsupported binary PCD field in {path}")
                formats.append(code)
            dtype = np.dtype({"names": fields, "formats": formats})
            packed = np.frombuffer(handle.read(), dtype=dtype, count=point_count)
            values = np.column_stack([packed[name] for name in fields]).astype(np.float64)
        else:
            raise ValueError(f"Unsupported PCD encoding {encoding}: {path}")
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


def crop_xy(fields: list[str], values: np.ndarray, center: np.ndarray, radius: float) -> np.ndarray:
    if radius <= 0:
        return values
    x = values[:, fields.index("x")]
    y = values[:, fields.index("y")]
    keep = (x - center[0]) ** 2 + (y - center[1]) ** 2 <= radius ** 2
    return values[keep]


class AuditPublisher(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("ddlzd_mapping_audit_viewer")
        reference_fields, reference = read_pcd(args.reference)
        direct_fields, direct = read_pcd(args.direct)
        ground_fields, ground = read_pcd(args.ground)
        change_fields, changes = read_pcd(args.changes)
        reference_xyz = reference[:, [reference_fields.index(name) for name in ("x", "y", "z")]]
        self.origin = np.median(reference_xyz, axis=0)
        roi_center = np.array([args.center_x, args.center_y])
        reference = crop_xy(reference_fields, reference, roi_center, args.radius)
        direct = crop_xy(direct_fields, direct, roi_center, args.radius)
        ground = crop_xy(ground_fields, ground, roi_center, args.radius)
        changes = crop_xy(change_fields, changes, roi_center, args.radius)
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
            ("/ddlzd/audit/direct_pointcloud", direct_fields, direct),
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
        self.get_logger().info(
            f"Local origin: {self.origin[0]:.3f}, {self.origin[1]:.3f}, {self.origin[2]:.3f}; "
            f"ROI center: {args.center_x:.3f}, {args.center_y:.3f}, radius {args.radius:.1f} m"
        )

def main() -> int:
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference", type=Path,
                        default=root / "pcds/696_5328_cropped_ground_only.pcd")
    parser.add_argument("--ground", type=Path,
                        default=root / "pcds/lidar_mapping_accuracy/update_only_ground_map.pcd")
    parser.add_argument("--direct", type=Path,
                        default=root / "pcds/lidar_mapping_accuracy/direct_corrected_pointcloud.pcd")
    parser.add_argument("--changes", type=Path,
                        default=root / "pcds/lidar_mapping_accuracy/change_only_obstacles.pcd")
    parser.add_argument("--maximum-reference-points", type=int, default=250000)
    parser.add_argument("--center-x", type=float, default=696496.704718,
                        help="UTM center easting; default is the grounded bag start")
    parser.add_argument("--center-y", type=float, default=5328128.7768,
                        help="UTM center northing; default is the grounded bag start")
    parser.add_argument("--radius", type=float, default=100.0,
                        help="Display radius in metres; use 0 for the complete map")
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
