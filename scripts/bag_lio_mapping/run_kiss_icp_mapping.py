#!/usr/bin/env python3
"""Run timestamp-correct KISS-ICP and reconstruct a bag-only deskewed point map."""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
import yaml
from kiss_icp.config import KISSConfig
from kiss_icp.pipeline import KissICP
from rosbags.highlevel import AnyReader

SCRIPTS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPTS))

from bag_lio_mapping.pcd_io import BinaryPcdWriter  # noqa: E402


POINT_DTYPE = np.dtype({
    "names": ["x", "y", "z", "intensity", "ring", "time"],
    "formats": ["<f4", "<f4", "<f4", "<f4", "<u2", "<f4"],
    "offsets": [0, 4, 8, 12, 16, 18],
    "itemsize": 22,
})


def quaternion_matrix(quaternion_xyzw: list[float]) -> np.ndarray:
    x, y, z, w = np.asarray(quaternion_xyzw, dtype=np.float64)
    norm = x * x + y * y + z * z + w * w
    scale = 2.0 / norm
    return np.array([
        [1.0 - scale * (y * y + z * z), scale * (x * y - z * w), scale * (x * z + y * w)],
        [scale * (x * y + z * w), 1.0 - scale * (x * x + z * z), scale * (y * z - x * w)],
        [scale * (x * z - y * w), scale * (y * z + x * w), 1.0 - scale * (x * x + y * y)],
    ])


def odometry_trajectory(bag: Path, topic: str) -> tuple[np.ndarray, np.ndarray]:
    times: list[float] = []
    poses: list[np.ndarray] = []
    with AnyReader([bag]) as reader:
        connections = [item for item in reader.connections if item.topic == topic]
        if len(connections) != 1:
            raise RuntimeError(f"Expected one connection for {topic}, found {len(connections)}")
        for _, timestamp_ns, raw in reader.messages(connections=connections):
            message = reader.deserialize(raw, connections[0].msgtype)
            position = message.pose.pose.position
            orientation = message.pose.pose.orientation
            pose = np.eye(4)
            pose[:3, :3] = quaternion_matrix([
                orientation.x, orientation.y, orientation.z, orientation.w,
            ])
            pose[:3, 3] = [position.x, position.y, position.z]
            times.append(timestamp_ns * 1e-9)
            poses.append(pose)
    return np.asarray(times), np.asarray(poses)


def interpolate_odometry(times: np.ndarray, poses: np.ndarray, timestamp: float) -> np.ndarray:
    right = int(np.searchsorted(times, timestamp, side="left"))
    right = min(max(right, 1), len(times) - 1)
    left = right - 1
    fraction = np.clip((timestamp - times[left]) / (times[right] - times[left]), 0.0, 1.0)
    # Rotation interpolation through SVD keeps this dependency-light and on SO(3).
    matrix = (1.0 - fraction) * poses[left, :3, :3] + fraction * poses[right, :3, :3]
    u, _, vt = np.linalg.svd(matrix)
    rotation = u @ vt
    if np.linalg.det(rotation) < 0.0:
        u[:, -1] *= -1.0
        rotation = u @ vt
    result = np.eye(4)
    result[:3, :3] = rotation
    result[:3, 3] = (
        (1.0 - fraction) * poses[left, :3, 3] + fraction * poses[right, :3, 3]
    )
    return result


def rotation_angle(transform: np.ndarray) -> float:
    cosine = np.clip((np.trace(transform[:3, :3]) - 1.0) * 0.5, -1.0, 1.0)
    return float(np.arccos(cosine))


def smooth_transform(previous: np.ndarray, current: np.ndarray, alpha: float) -> np.ndarray:
    result = np.eye(4)
    result[:3, 3] = (1.0 - alpha) * previous[:3, 3] + alpha * current[:3, 3]
    matrix = (1.0 - alpha) * previous[:3, :3] + alpha * current[:3, :3]
    u, _, vt = np.linalg.svd(matrix)
    result[:3, :3] = u @ vt
    if np.linalg.det(result[:3, :3]) < 0.0:
        u[:, -1] *= -1.0
        result[:3, :3] = u @ vt
    return result


def valid_body_scan(message, rotation: np.ndarray, translation: np.ndarray,
                    minimum_range: float, maximum_range: float) -> tuple[np.ndarray, np.ndarray]:
    if message.point_step != POINT_DTYPE.itemsize:
        raise RuntimeError(f"Expected 22-byte Velodyne records, got {message.point_step}")
    values = np.frombuffer(message.data, dtype=POINT_DTYPE,
                           count=int(message.width) * int(message.height))
    lidar = np.column_stack((values["x"], values["y"], values["z"])).astype(np.float64)
    timestamps = values["time"].astype(np.float64)
    ranges = np.linalg.norm(lidar, axis=1)
    keep = (
        np.isfinite(lidar).all(axis=1) & np.isfinite(timestamps)
        & (ranges >= minimum_range) & (ranges <= maximum_range)
    )
    lidar, timestamps = lidar[keep], timestamps[keep]
    body = (rotation @ lidar.T).T + translation
    return body, timestamps


def main() -> int:
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--topic", default="/velodyne_points")
    parser.add_argument("--config", type=Path,
                        default=root / "scripts/bag_lio_mapping/kiss_icp_config.yaml")
    parser.add_argument("--output", type=Path,
                        default=root / "pcds/bag_only_kiss_icp_reconstructed.pcd")
    parser.add_argument("--poses", type=Path,
                        default=root / "pcds/bag_only_kiss_icp_poses.npz")
    parser.add_argument("--diagnostics", type=Path,
                        default=root / "pcds/bag_only_kiss_icp_diagnostics.json")
    parser.add_argument("--max-scans", type=int, default=0)
    parser.add_argument("--disable-deskew", action="store_true")
    args = parser.parse_args()
    settings = yaml.safe_load(args.config.read_text())

    config = KISSConfig()
    config.data.min_range = float(settings["data"]["min_range_m"])
    config.data.max_range = float(settings["data"]["max_range_m"])
    config.data.deskew = bool(settings["data"]["deskew"])
    if args.disable_deskew:
        config.data.deskew = False
    config.registration.max_num_iterations = int(settings["registration"]["max_num_iterations"])
    config.registration.convergence_criterion = float(
        settings["registration"]["convergence_criterion"]
    )
    config.registration.max_num_threads = int(settings["registration"]["max_num_threads"])
    config.mapping.voxel_size = float(settings["mapping"]["voxel_size_m"])
    config.mapping.max_points_per_voxel = int(settings["mapping"]["max_points_per_voxel"])
    threshold = settings["adaptive_threshold"]
    config.adaptive_threshold.fixed_threshold = float(threshold["fixed_threshold_m"])
    config.adaptive_threshold.initial_threshold = float(threshold["initial_threshold_m"])
    config.adaptive_threshold.min_motion_th = float(threshold["minimum_motion_m"])
    mapper = KissICP(config)
    odom_times, odom_poses = odometry_trajectory(args.bag, settings["data"]["odometry_topic"])

    extrinsic = settings["extrinsic_base_from_lidar"]
    rotation = quaternion_matrix(extrinsic["quaternion_xyzw"])
    translation = np.asarray(extrinsic["translation_m"], dtype=np.float64)
    output_stride = int(settings["reconstruction"]["point_stride"])
    writer = BinaryPcdWriter(args.output)
    poses: list[np.ndarray] = []
    frame_times: list[float] = []
    input_counts: list[int] = []
    corrections: list[tuple[float, float]] = []
    rejected_registrations = 0
    first_prior: np.ndarray | None = None
    previous_prior = np.eye(4)
    filtered_map_from_odom = np.eye(4)
    start = time.monotonic()

    with AnyReader([args.bag]) as reader:
        connections = [item for item in reader.connections if item.topic == args.topic]
        if len(connections) != 1:
            raise RuntimeError(f"Expected one connection for {args.topic}, found {len(connections)}")
        for _, timestamp_ns, raw in reader.messages(connections=connections):
            if args.max_scans and len(poses) >= args.max_scans:
                break
            message = reader.deserialize(raw, connections[0].msgtype)
            points, timestamps = valid_body_scan(
                message, rotation, translation, config.data.min_range, config.data.max_range,
            )
            if len(points) < 100:
                continue
            scan_time = timestamp_ns * 1e-9
            odometry_pose = interpolate_odometry(odom_times, odom_poses, scan_time)
            if first_prior is None:
                first_prior = odometry_pose
            prior = np.linalg.inv(first_prior) @ odometry_pose
            prior_delta = np.linalg.inv(previous_prior) @ prior
            deskewed = mapper.preprocessor.preprocess(points, timestamps, prior_delta)
            source, frame_downsample = mapper.voxelize(deskewed)
            sigma = mapper.adaptive_threshold.get_threshold()
            candidate = mapper.registration.align_points_to_map(
                points=source,
                voxel_map=mapper.local_map,
                initial_guess=prior,
                max_correspondance_distance=3.0 * sigma,
                kernel=sigma,
            )
            innovation = np.linalg.inv(prior) @ candidate
            translation_error = float(np.linalg.norm(innovation[:3, 3]))
            rotation_error = rotation_angle(innovation)
            gate = settings["innovation_gate"]
            accepted = (
                translation_error <= float(gate["maximum_translation_m"])
                and np.degrees(rotation_error) <= float(gate["maximum_rotation_deg"])
            )
            if accepted:
                candidate_map_from_odom = candidate @ np.linalg.inv(prior)
                filtered_map_from_odom = smooth_transform(
                    filtered_map_from_odom,
                    candidate_map_from_odom,
                    float(gate["correction_smoothing_alpha"]),
                )
            pose = filtered_map_from_odom @ prior
            if accepted:
                mapper.adaptive_threshold.update_model_deviation(innovation)
            else:
                rejected_registrations += 1
            mapper.local_map.update(frame_downsample, pose)
            mapper.last_delta = np.linalg.inv(mapper.last_pose) @ pose
            mapper.last_pose = pose
            previous_prior = prior
            corrections.append((translation_error, np.degrees(rotation_error)))
            reconstructed = (pose[:3, :3] @ deskewed[::output_stride].T).T + pose[:3, 3]
            writer.write(reconstructed)
            poses.append(pose)
            frame_times.append(scan_time)
            input_counts.append(len(points))
            if len(poses) % 250 == 0:
                print(f"Registered {len(poses)} scans; wrote {writer.count} points", flush=True)
    writer.close()

    pose_array = np.asarray(poses)
    times = np.asarray(frame_times)
    np.savez_compressed(args.poses, poses=pose_array, timestamps=times)
    step_distance = np.linalg.norm(np.diff(pose_array[:, :3, 3], axis=0), axis=1)
    elapsed = time.monotonic() - start
    correction_array = np.asarray(corrections)
    diagnostics = {
        "method": "KISS-ICP scan-to-local-map odometry with per-point deskew",
        "backend_version": "kiss-icp 1.2.3",
        "uses_bayern_reference": False,
        "uses_imu": False,
        "uses_onboard_odometry_prior": True,
        "deskew_enabled": config.data.deskew,
        "scans": len(poses),
        "reconstructed_points": writer.count,
        "valid_points_per_scan": {
            "median": float(np.median(input_counts)),
            "minimum": int(np.min(input_counts)),
            "maximum": int(np.max(input_counts)),
        },
        "trajectory_path_length_m": float(np.sum(step_distance)),
        "trajectory_end_translation_m": pose_array[-1, :3, 3].tolist(),
        "registration_innovation": {
            "rejected_scans": rejected_registrations,
            "translation_median_m": float(np.median(correction_array[:, 0])),
            "translation_p95_m": float(np.percentile(correction_array[:, 0], 95)),
            "rotation_median_deg": float(np.median(correction_array[:, 1])),
            "rotation_p95_deg": float(np.percentile(correction_array[:, 1], 95)),
        },
        "runtime_seconds": elapsed,
        "average_scans_per_second": len(poses) / elapsed,
        "config": settings,
        "limitations": [
            "This is odometry-seeded LiDAR scan-to-local-map optimization, not tightly coupled LIO.",
            "No global loop closure, GNSS factor graph, or global pose-graph optimization is applied yet.",
            "The reconstructed map uses online optimized poses; a later global graph must rebuild it again.",
        ],
    }
    args.diagnostics.write_text(json.dumps(diagnostics, indent=2) + "\n")
    print(json.dumps(diagnostics, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
