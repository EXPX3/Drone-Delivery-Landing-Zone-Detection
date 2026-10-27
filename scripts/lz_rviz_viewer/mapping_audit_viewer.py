#!/usr/bin/env python3
"""Publish mapping-audit PCD layers for interactive RViz inspection."""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np
import rclpy
from scipy.spatial import cKDTree
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


def cloud_message(fields: list[str], values: np.ndarray, origin: np.ndarray,
                  intensity_override: np.ndarray | None = None) -> PointCloud2:
    indices = [fields.index(name) for name in ("x", "y", "z")]
    xyz = values[:, indices] - origin
    if intensity_override is not None:
        intensity = np.asarray(intensity_override, dtype=np.float64)
        if len(intensity) != len(values):
            raise ValueError("Intensity override must have one value per point")
    elif "count" in fields:
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


def orthophoto_message(path: Path, bounds: tuple[float, float, float, float],
                       elevation: float, origin: np.ndarray, stride: int) -> PointCloud2:
    image = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f"Could not read orthophoto: {path}")
    height, width = image.shape[:2]
    rows = np.arange(0, height, stride, dtype=np.int32)
    cols = np.arange(0, width, stride, dtype=np.int32)
    cc, rr = np.meshgrid(cols, rows)
    min_x, min_y, max_x, max_y = bounds
    x = min_x + cc.ravel() / max(1, width - 1) * (max_x - min_x)
    y = max_y - rr.ravel() / max(1, height - 1) * (max_y - min_y)
    bgr = image[rr.ravel(), cc.ravel()]
    rgb_bits = ((bgr[:, 2].astype(np.uint32) << 16)
                | (bgr[:, 1].astype(np.uint32) << 8)
                | bgr[:, 0].astype(np.uint32))
    packed = np.empty(len(x), dtype=[("x", "<f4"), ("y", "<f4"),
                                     ("z", "<f4"), ("rgb", "<u4")])
    packed["x"] = x - origin[0]
    packed["y"] = y - origin[1]
    packed["z"] = elevation - origin[2]
    packed["rgb"] = rgb_bits
    message = PointCloud2()
    message.header = Header(frame_id="lz_world")
    message.height = 1
    message.width = len(packed)
    message.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1),
    ]
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
        if args.standalone is not None:
            direct_fields, direct = read_pcd(args.standalone)
            direct_xyz = direct[:, [direct_fields.index(name) for name in ("x", "y", "z")]]
            self.origin = np.median(direct_xyz, axis=0)
            direct = crop_xy(
                direct_fields, direct, np.array([args.center_x, args.center_y]), args.radius,
            )
            if len(direct) > args.maximum_direct_points:
                index = np.random.default_rng(31).choice(
                    len(direct), args.maximum_direct_points, replace=False,
                )
                direct = direct[index]
            qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST, depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            publisher = self.create_publisher(PointCloud2, "/ddlzd/audit/direct_pointcloud", qos)
            message = cloud_message(direct_fields, direct, self.origin)
            publisher.publish(message)
            self._layer_publishers = [publisher]
            self._layer_messages = [message]
            self.get_logger().info(
                f"Published {len(direct)} standalone points from {args.standalone}"
            )
            self.get_logger().info(
                f"Local origin: {self.origin[0]:.3f}, {self.origin[1]:.3f}, "
                f"{self.origin[2]:.3f}; ROI center: {args.center_x:.3f}, "
                f"{args.center_y:.3f}, radius {args.radius:.1f} m"
            )
            return
        reference_fields, reference = read_pcd(args.reference)
        direct_fields, direct = read_pcd(args.direct)
        ground_fields, ground = read_pcd(args.ground)
        change_fields, changes = read_pcd(args.changes)
        reference_xyz = reference[:, [reference_fields.index(name) for name in ("x", "y", "z")]]
        self.origin = np.median(reference_xyz, axis=0)
        reference_bounds = (
            float(reference_xyz[:, 0].min() - args.orthophoto_margin),
            float(reference_xyz[:, 1].min() - args.orthophoto_margin),
            float(reference_xyz[:, 0].max() + args.orthophoto_margin),
            float(reference_xyz[:, 1].max() + args.orthophoto_margin),
        )
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
        if len(direct) > args.maximum_direct_points:
            index = np.random.default_rng(37).choice(
                len(direct), args.maximum_direct_points, replace=False,
            )
            direct = direct[index]
        if not len(ground):
            raise ValueError("Corrected ground map is empty inside the selected ROI")
        direct_xyz = direct[:, [direct_fields.index(name) for name in ("x", "y", "z")]]
        ground_xyz = ground[:, [ground_fields.index(name) for name in ("x", "y", "z")]]
        _, nearest_ground = cKDTree(ground_xyz[:, :2]).query(direct_xyz[:, :2], workers=-1)
        direct_height_agl = np.maximum(0.0, direct_xyz[:, 2] - ground_xyz[nearest_ground, 2])
        self.get_logger().info(
            "Full-scene height above corrected ground: "
            f"median {np.median(direct_height_agl):.2f} m, "
            f"P95 {np.percentile(direct_height_agl, 95):.2f} m, "
            f"maximum {np.max(direct_height_agl):.2f} m"
        )
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        layers = [
            ("/ddlzd/audit/reference_ground", reference_fields, reference, None),
            ("/ddlzd/audit/direct_pointcloud", direct_fields, direct, direct_height_agl),
            ("/ddlzd/audit/corrected_ground", ground_fields, ground, None),
            ("/ddlzd/audit/scene_changes", change_fields, changes, None),
        ]
        self._layer_publishers = []
        self._layer_messages = []
        for topic, fields, values, intensity in layers:
            publisher = self.create_publisher(PointCloud2, topic, qos)
            message = cloud_message(fields, values, self.origin, intensity)
            publisher.publish(message)
            self._layer_publishers.append(publisher)
            self._layer_messages.append(message)
            self.get_logger().info(f"Published {len(values)} points on {topic}")
        if args.orthophoto.is_file():
            publisher = self.create_publisher(PointCloud2, "/ddlzd/audit/orthophoto", qos)
            message = orthophoto_message(
                args.orthophoto, reference_bounds,
                float(reference_xyz[:, 2].min() - args.orthophoto_z_offset),
                self.origin, args.orthophoto_stride,
            )
            publisher.publish(message)
            self._layer_publishers.append(publisher)
            self._layer_messages.append(message)
            self.get_logger().info(
                f"Published {message.width} RGB orthophoto samples on /ddlzd/audit/orthophoto"
            )
        self.get_logger().info(
            f"Local origin: {self.origin[0]:.3f}, {self.origin[1]:.3f}, {self.origin[2]:.3f}; "
            f"ROI center: {args.center_x:.3f}, {args.center_y:.3f}, radius {args.radius:.1f} m"
        )

def main() -> int:
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--standalone", type=Path,
                        help="Display one local-frame PCD without loading audit reference layers")
    parser.add_argument("--reference", type=Path,
                        default=root / "pcds/696_5328_cropped_by_bayernatlas_20260812084517.pcd",
                        help="Georeferenced Bayern reference PCD")
    parser.add_argument("--ground", type=Path,
                        default=root / "pcds/lidar_mapping_accuracy/update_only_ground_map.pcd")
    parser.add_argument("--direct", type=Path,
                        default=root / "pcds/lidar_mapping_accuracy/direct_corrected_pointcloud.pcd")
    parser.add_argument("--changes", type=Path,
                        default=root / "pcds/lidar_mapping_accuracy/change_only_obstacles.pcd")
    parser.add_argument("--orthophoto", type=Path, default=root / "pcds/696_5328_cropped_by_bayernatlas_20260812084517_satellite_overlay_dop20.png")
    parser.add_argument("--orthophoto-stride", type=int, default=4)
    parser.add_argument("--orthophoto-margin", type=float, default=10.0)
    parser.add_argument("--orthophoto-z-offset", type=float, default=0.35)
    parser.add_argument("--maximum-reference-points", type=int, default=250000)
    parser.add_argument("--maximum-direct-points", type=int, default=1000000)
    parser.add_argument("--center-x", type=float,
                        help="ROI center X; defaults to zero for standalone or grounded UTM start")
    parser.add_argument("--center-y", type=float,
                        help="ROI center Y; defaults to zero for standalone or grounded UTM start")
    parser.add_argument("--radius", type=float, default=100.0,
                        help="Display radius in metres; use 0 for the complete map")
    args = parser.parse_args()
    if args.center_x is None:
        args.center_x = 0.0 if args.standalone is not None else 696496.704718
    if args.center_y is None:
        args.center_y = 0.0 if args.standalone is not None else 5328128.7768
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
