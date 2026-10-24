#!/usr/bin/env python3
"""Estimate grounded end-to-start LiDAR closure and globally correct scan poses."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import small_gicp
import yaml
from rosbags.highlevel import AnyReader

SCRIPTS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPTS / "bag_lio_mapping"))

from run_kiss_icp_mapping import quaternion_matrix, valid_body_scan  # noqa: E402


def rotation_fraction(rotation: np.ndarray, fraction: float) -> np.ndarray:
    cosine = np.clip((np.trace(rotation) - 1.0) * 0.5, -1.0, 1.0)
    angle = float(np.arccos(cosine))
    if angle < 1e-12:
        return np.eye(3)
    axis = np.array([
        rotation[2, 1] - rotation[1, 2],
        rotation[0, 2] - rotation[2, 0],
        rotation[1, 0] - rotation[0, 1],
    ]) / (2.0 * np.sin(angle))
    skew = np.array([[0.0, -axis[2], axis[1]],
                     [axis[2], 0.0, -axis[0]],
                     [-axis[1], axis[0], 0.0]])
    theta = fraction * angle
    return np.eye(3) + np.sin(theta) * skew + (1.0 - np.cos(theta)) * (skew @ skew)


def distribute_closure(poses: np.ndarray, closure: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    distance = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(poses[:, :3, 3], axis=0), axis=1))]
    fraction = distance / max(distance[-1], 1e-12)
    corrected = poses.copy()
    for index, alpha in enumerate(fraction):
        correction = np.eye(4)
        correction[:3, :3] = rotation_fraction(closure[:3, :3], float(alpha))
        correction[:3, 3] = alpha * closure[:3, 3]
        corrected[index] = correction @ poses[index]
    return corrected, fraction


def main() -> int:
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--poses", type=Path,
                        default=root / "pcds/bag_only_gnss_scaled_poses.npz")
    parser.add_argument("--config", type=Path,
                        default=root / "scripts/bag_lio_mapping/kiss_icp_config.yaml")
    parser.add_argument("--output", type=Path,
                        default=root / "pcds/bag_only_loop_optimized_poses.npz")
    parser.add_argument("--diagnostics", type=Path,
                        default=root / "pcds/bag_only_loop_closure_diagnostics.json")
    args = parser.parse_args()
    settings = yaml.safe_load(args.config.read_text())
    pose_data = np.load(args.poses)
    poses, pose_times = pose_data["poses"], pose_data["timestamps"]
    loop = settings["loop_closure"]
    endpoint_scans = int(loop["endpoint_scans"])
    scan_stride = int(loop["scan_stride"])
    point_stride = int(loop["point_stride"])
    extrinsic = settings["extrinsic_base_from_lidar"]
    rotation = quaternion_matrix(extrinsic["quaternion_xyzw"])
    translation = np.asarray(extrinsic["translation_m"], dtype=np.float64)

    start_points: list[np.ndarray] = []
    end_points: list[np.ndarray] = []
    index = 0
    with AnyReader([args.bag]) as reader:
        connections = [item for item in reader.connections if item.topic == "/velodyne_points"]
        for _, _, raw in reader.messages(connections=connections):
            if index >= len(poses):
                break
            is_start = index < endpoint_scans
            is_end = index >= len(poses) - endpoint_scans
            if (is_start or is_end) and index % scan_stride == 0:
                message = reader.deserialize(raw, connections[0].msgtype)
                points, _ = valid_body_scan(message, rotation, translation, 1.0, 60.0)
                points = points[::point_stride]
                pose = poses[index]
                mapped = (pose[:3, :3] @ points.T).T + pose[:3, 3]
                (start_points if is_start else end_points).append(mapped)
            index += 1
    target, source = np.vstack(start_points), np.vstack(end_points)
    registration = small_gicp.align(
        target, source, init_T_target_source=np.eye(4), registration_type="GICP",
        downsampling_resolution=float(loop["downsampling_resolution_m"]),
        max_correspondence_distance=float(loop["maximum_correspondence_m"]),
        num_threads=12, max_iterations=50,
    )
    closure = np.asarray(registration.T_target_source)
    optimized, fraction = distribute_closure(poses, closure)
    np.savez_compressed(args.output, poses=optimized, timestamps=pose_times,
                        closure=closure, correction_fraction=fraction)
    end_before = poses[-1, :3, 3]
    end_after = optimized[-1, :3, 3]
    result = {
        "method": "grounded endpoint GICP closure with distributed pose-graph correction",
        "uses_bayern_reference": False,
        "converged": bool(registration.converged),
        "inliers": int(registration.num_inliers),
        "registration_error": float(registration.error),
        "closure_transform": closure.tolist(),
        "end_translation_before_m": end_before.tolist(),
        "end_translation_after_m": end_after.tolist(),
        "end_translation_norm_before_m": float(np.linalg.norm(end_before)),
        "end_translation_norm_after_m": float(np.linalg.norm(end_after)),
        "limitation": (
            "One verified endpoint loop is used with uniform odometry-edge information; "
            "future work should add intermediate loop candidates and covariance weighting."
        ),
    }
    args.diagnostics.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
