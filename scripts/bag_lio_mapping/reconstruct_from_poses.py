#!/usr/bin/env python3
"""Reconstruct a PCD using globally optimized scan poses and per-point timing."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import yaml
from rosbags.highlevel import AnyReader

SCRIPTS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPTS / "bag_lio_mapping"))

from pcd_io import BinaryPcdWriter  # noqa: E402
from run_kiss_icp_mapping import quaternion_matrix, valid_body_scan  # noqa: E402


def rotation_axis_angle(rotation: np.ndarray) -> tuple[np.ndarray, float]:
    angle = float(np.arccos(np.clip((np.trace(rotation) - 1.0) * 0.5, -1.0, 1.0)))
    if angle < 1e-12:
        return np.array([1.0, 0.0, 0.0]), 0.0
    axis = np.array([rotation[2, 1] - rotation[1, 2],
                     rotation[0, 2] - rotation[2, 0],
                     rotation[1, 0] - rotation[0, 1]]) / (2.0 * np.sin(angle))
    return axis, angle


def transform_timed_points(points: np.ndarray, relative_times: np.ndarray,
                           previous: np.ndarray, current: np.ndarray) -> np.ndarray:
    minimum, maximum = float(np.min(relative_times)), float(np.max(relative_times))
    phase = (relative_times - minimum) / max(maximum - minimum, 1e-12)
    position = previous[:3, 3] * (1.0 - phase[:, None]) + current[:3, 3] * phase[:, None]
    relative_rotation = previous[:3, :3].T @ current[:3, :3]
    axis, angle = rotation_axis_angle(relative_rotation)
    theta = phase * angle
    cross = np.cross(np.broadcast_to(axis, points.shape), points)
    dot = points @ axis
    rotated_relative = (
        points * np.cos(theta)[:, None]
        + cross * np.sin(theta)[:, None]
        + axis[None, :] * dot[:, None] * (1.0 - np.cos(theta))[:, None]
    )
    return (previous[:3, :3] @ rotated_relative.T).T + position


def main() -> int:
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--poses", type=Path,
                        default=root / "pcds/bag_only_loop_optimized_poses.npz")
    parser.add_argument("--config", type=Path,
                        default=root / "scripts/bag_lio_mapping/kiss_icp_config.yaml")
    parser.add_argument("--output", type=Path,
                        default=root / "pcds/bag_only_loop_optimized_reconstructed.pcd")
    parser.add_argument("--diagnostics", type=Path,
                        default=root / "pcds/bag_only_loop_optimized_reconstruction.json")
    args = parser.parse_args()
    settings = yaml.safe_load(args.config.read_text())
    pose_data = np.load(args.poses)
    poses = pose_data["poses"]
    extrinsic = settings["extrinsic_base_from_lidar"]
    rotation = quaternion_matrix(extrinsic["quaternion_xyzw"])
    translation = np.asarray(extrinsic["translation_m"], dtype=np.float64)
    stride = int(settings["reconstruction"]["point_stride"])
    writer = BinaryPcdWriter(args.output)
    scans = 0
    previous = poses[0]
    with AnyReader([args.bag]) as reader:
        connections = [item for item in reader.connections if item.topic == "/velodyne_points"]
        for _, _, raw in reader.messages(connections=connections):
            if scans >= len(poses):
                break
            message = reader.deserialize(raw, connections[0].msgtype)
            points, relative_times = valid_body_scan(message, rotation, translation, 1.0, 80.0)
            current = poses[scans]
            mapped = transform_timed_points(points, relative_times, previous, current)
            writer.write(mapped[::stride])
            previous = current
            scans += 1
            if scans % 500 == 0:
                print(f"Reconstructed {scans} scans; wrote {writer.count} points", flush=True)
    writer.close()
    result = {
        "method": "per-point interpolation of supplied optimized scan poses",
        "uses_bayern_reference": False,
        "scans": scans,
        "points": writer.count,
        "point_stride": stride,
        "poses": str(args.poses),
        "output": str(args.output),
    }
    args.diagnostics.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
