#!/usr/bin/env python3
"""Build a bag-only point map using per-point LiDAR time and onboard poses."""

from __future__ import annotations

import argparse
import json
import shutil
import sys
from pathlib import Path

import numpy as np
import yaml

SCRIPTS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPTS))

from project_lz_into_bag_camera import (  # noqa: E402
    chain_transform, open_reader, read_pose_and_gps, require_ros, stamp_ns,
)


POINT_DTYPE = np.dtype({
    "names": ["x", "y", "z", "intensity", "ring", "time"],
    "formats": ["<f4", "<f4", "<f4", "<f4", "<u2", "<f4"],
    "offsets": [0, 4, 8, 12, 16, 18],
    "itemsize": 22,
})


class BinaryPcdWriter:
    def __init__(self, path: Path) -> None:
        self.path = path
        self.temporary = path.with_suffix(path.suffix + ".points.tmp")
        self.handle = self.temporary.open("wb")
        self.count = 0

    def write(self, points: np.ndarray) -> None:
        xyz = np.ascontiguousarray(points, dtype=np.float32)
        self.handle.write(xyz.tobytes())
        self.count += len(xyz)

    def close(self) -> None:
        self.handle.close()
        header = (
            "# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z\n"
            "SIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
            f"WIDTH {self.count}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
            f"POINTS {self.count}\nDATA binary\n"
        )
        with self.path.open("wb") as destination, self.temporary.open("rb") as source:
            destination.write(header.encode("ascii"))
            shutil.copyfileobj(source, destination, length=4 * 1024 * 1024)
        self.temporary.unlink()


def rotate_vectors(quaternions: np.ndarray, vectors: np.ndarray) -> np.ndarray:
    xyz = quaternions[:, :3]
    uv = np.cross(xyz, vectors)
    uuv = np.cross(xyz, uv)
    return vectors + 2.0 * (quaternions[:, 3:4] * uv + uuv)


def interpolate_poses(times: np.ndarray, translations: np.ndarray, quaternions: np.ndarray,
                      query: np.ndarray, maximum_gap_ns: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    right = np.searchsorted(times, query, side="left")
    valid = (right > 0) & (right < len(times))
    right = np.clip(right, 1, len(times) - 1)
    left = right - 1
    intervals = times[right] - times[left]
    valid &= intervals <= maximum_gap_ns
    fraction = np.clip((query - times[left]) / np.maximum(intervals, 1), 0.0, 1.0)
    position = translations[left] * (1.0 - fraction[:, None]) + translations[right] * fraction[:, None]
    q0 = quaternions[left]
    q1 = quaternions[right].copy()
    q1[np.sum(q0 * q1, axis=1) < 0.0] *= -1.0
    rotation = q0 * (1.0 - fraction[:, None]) + q1 * fraction[:, None]
    rotation /= np.linalg.norm(rotation, axis=1, keepdims=True)
    return position, rotation, valid


def trajectory_arrays(tf_data: dict, parent: str, child: str) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    key = (parent, child)
    reverse = (child, parent)
    if key in tf_data["dynamic"]:
        samples = sorted(tf_data["dynamic"][key], key=lambda item: item[0])
    elif reverse in tf_data["dynamic"]:
        samples = [(timestamp, transform.inverse()) for timestamp, transform in tf_data["dynamic"][reverse]]
        samples.sort(key=lambda item: item[0])
    else:
        raise RuntimeError(f"No dynamic trajectory for {parent} -> {child}")
    return (
        np.asarray([item[0] for item in samples], dtype=np.int64),
        np.asarray([item[1].translation for item in samples], dtype=np.float64),
        np.asarray([item[1].quaternion_xyzw for item in samples], dtype=np.float64),
    )


def main() -> int:
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--config", type=Path, default=root / "scripts/bag_lio_mapping/config.yaml")
    parser.add_argument("--output", type=Path, default=root / "pcds/bag_only_deskewed_map.pcd")
    parser.add_argument("--diagnostics", type=Path,
                        default=root / "pcds/bag_only_deskewed_map_diagnostics.json")
    parser.add_argument("--max-scans", type=int, default=0)
    parser.add_argument("--disable-deskew", action="store_true",
                        help="Use one header-time pose per scan for a matched control map")
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text())

    camera_config = {
        "topics": {"tf": config["topics"]["tf"], "tf_static": config["topics"]["tf_static"],
                   "gps": config["topics"]["gnss"]},
    }
    tf_data, _ = read_pose_and_gps(args.bag, camera_config)
    parent = config["frames"]["trajectory_parent"]
    body = config["frames"]["body"]
    pose_times, pose_translations, pose_quaternions = trajectory_arrays(tf_data, parent, body)
    base_to_lidar = chain_transform(tf_data, config["frames"]["lidar_chain"],
                                    int(pose_times[0]), 0.25)
    base_lidar_rotation = base_to_lidar.matrix[:3, :3]
    base_lidar_translation = base_to_lidar.translation

    rosbag2_py, deserialize, get_message = require_ros()
    reader = open_reader(args.bag)
    topic = config["topics"]["points"]
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    message_type = get_message(topic_types[topic])
    mapping = config["mapping"]
    stride = int(mapping["point_stride"])
    minimum_range = float(mapping["minimum_range_m"])
    maximum_range = float(mapping["maximum_range_m"])
    maximum_gap_ns = int(float(mapping["maximum_pose_gap_s"]) * 1e9)
    writer = BinaryPcdWriter(args.output)

    scans = skipped_scans = rejected_points = 0
    time_min = np.inf
    time_max = -np.inf
    while reader.has_next() and (not args.max_scans or scans < args.max_scans):
        _, raw, bag_time = reader.read_next()
        message = deserialize(raw, message_type)
        if message.point_step != POINT_DTYPE.itemsize:
            raise RuntimeError(f"Expected 22-byte Velodyne points, got {message.point_step}")
        values = np.frombuffer(message.data, dtype=POINT_DTYPE,
                               count=int(message.width) * int(message.height))[::stride]
        sensor = np.column_stack((values["x"], values["y"], values["z"])).astype(np.float64)
        relative_time = values["time"].astype(np.float64)
        finite = np.isfinite(sensor).all(axis=1) & np.isfinite(relative_time)
        ranges = np.linalg.norm(sensor, axis=1)
        keep = finite & (ranges >= minimum_range) & (ranges <= maximum_range)
        sensor = sensor[keep]
        relative_time = relative_time[keep]
        if not len(sensor):
            skipped_scans += 1
            continue
        header_time = stamp_ns(message, bag_time)
        query = header_time + np.rint(relative_time * 1e9).astype(np.int64)
        if args.disable_deskew:
            query.fill(header_time)
        position, rotation, valid = interpolate_poses(
            pose_times, pose_translations, pose_quaternions, query, maximum_gap_ns,
        )
        rejected_points += int(np.count_nonzero(~valid))
        if np.count_nonzero(valid) < 20:
            skipped_scans += 1
            continue
        base_points = (base_lidar_rotation @ sensor[valid].T).T + base_lidar_translation
        mapped = rotate_vectors(rotation[valid], base_points) + position[valid]
        writer.write(mapped)
        time_min = min(time_min, float(np.min(relative_time)))
        time_max = max(time_max, float(np.max(relative_time)))
        scans += 1
        if scans % 250 == 0:
            print(f"Deskewed {scans} scans and wrote {writer.count} points", flush=True)
    writer.close()
    diagnostics = {
        "method": ("single-pose control" if args.disable_deskew
                   else "per-point onboard-pose deskew baseline"),
        "deskew_enabled": not args.disable_deskew,
        "uses_bayern_reference": False,
        "output": str(args.output),
        "scans": scans,
        "skipped_scans": skipped_scans,
        "points": writer.count,
        "points_rejected_for_pose_gap": rejected_points,
        "point_stride": stride,
        "point_relative_time_seconds": {"minimum": time_min, "maximum": time_max},
        "pose_samples": len(pose_times),
        "base_to_lidar_translation_m": base_lidar_translation.tolist(),
        "base_to_lidar_quaternion_xyzw": base_to_lidar.quaternion_xyzw.tolist(),
        "limitations": [
            "Uses recorded onboard map-to-base trajectory; it is not yet tightly coupled LIO.",
            "Quaternion interpolation is normalized linear interpolation between recorded poses.",
            "Dynamic objects are not yet removed.",
            "No Bayern points or elevations are used during map construction.",
        ],
    }
    args.diagnostics.write_text(json.dumps(diagnostics, indent=2) + "\n")
    print(json.dumps(diagnostics, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
