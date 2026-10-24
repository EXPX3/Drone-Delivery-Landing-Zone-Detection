#!/usr/bin/env python3
"""Evaluate frozen bag-only scan poses against held-out Bayern ground points."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import yaml
from pyproj import Transformer
from rosbags.highlevel import AnyReader
from scipy.spatial import cKDTree

SCRIPTS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPTS / "bag_lio_mapping"))

from reconstruct_from_poses import transform_timed_points  # noqa: E402
from run_kiss_icp_mapping import (  # noqa: E402
    interpolate_odometry,
    odometry_trajectory,
    quaternion_matrix,
    valid_body_scan,
)


def load_ascii_pcd_xyz(path: Path) -> np.ndarray:
    with path.open("rb") as handle:
        header_lines = 0
        while True:
            line = handle.readline()
            if not line:
                raise ValueError(f"Missing DATA line in {path}")
            header_lines += 1
            if line.startswith(b"DATA"):
                if line.strip() != b"DATA ascii":
                    raise ValueError("The held-out evaluator currently requires an ASCII PCD")
                break
    return np.loadtxt(path, skiprows=header_lines, usecols=(0, 1, 2))


def horizontal_gnss_registration(bag: Path, odom_times: np.ndarray,
                                 odom_poses: np.ndarray, topic: str):
    times, latitude, longitude = [], [], []
    with AnyReader([bag]) as reader:
        connections = [item for item in reader.connections if item.topic == topic]
        if not connections:
            raise ValueError(f"GNSS topic not found: {topic}")
        for _, timestamp, raw in reader.messages(connections=connections):
            message = reader.deserialize(raw, connections[0].msgtype)
            times.append(timestamp * 1e-9)
            latitude.append(message.latitude)
            longitude.append(message.longitude)
    times = np.asarray(times)
    latitude, longitude = np.asarray(latitude), np.asarray(longitude)
    zone = int((float(np.median(longitude)) + 180.0) // 6.0) + 1
    epsg = (32600 if float(np.median(latitude)) >= 0.0 else 32700) + zone
    transformer = Transformer.from_crs(4326, epsg, always_xy=True)
    east, north = transformer.transform(longitude, latitude)
    valid = (times >= odom_times[0]) & (times <= odom_times[-1])
    times = times[valid]
    gnss = np.column_stack((np.asarray(east)[valid], np.asarray(north)[valid]))
    odom = np.column_stack((
        np.interp(times, odom_times, odom_poses[:, 0, 3]),
        np.interp(times, odom_times, odom_poses[:, 1, 3]),
    ))
    centered_odom, centered_gnss = odom - odom.mean(0), gnss - gnss.mean(0)
    u, _, vt = np.linalg.svd(centered_odom.T @ centered_gnss)
    rotation = u @ vt
    if np.linalg.det(rotation) < 0.0:
        u[:, -1] *= -1.0
        rotation = u @ vt
    translation = gnss.mean(0) - odom.mean(0) @ rotation
    residual = odom @ rotation + translation - gnss
    return rotation, translation, float(np.sqrt(np.mean(np.sum(residual ** 2, axis=1)))), epsg


def robust_ground_bias(residual: np.ndarray) -> tuple[float, float, int]:
    if len(residual) <= 50:
        return np.nan, np.nan, 0
    low, high = np.percentile(residual, [1, 99])
    edges = np.arange(low, high + 0.05, 0.05)
    histogram, _ = np.histogram(residual, bins=edges)
    peak = int(np.argmax(histogram))
    mode = 0.5 * (edges[peak] + edges[peak + 1])
    selected = residual[np.abs(residual - mode) < 0.25]
    bias = float(np.median(selected))
    spread = float(1.4826 * np.median(np.abs(selected - bias)))
    return bias, spread, len(selected)


def main() -> int:
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--poses", type=Path, required=True)
    parser.add_argument("--reference", type=Path,
                        default=root / "pcds/696_5328_cropped_ground_only.pcd")
    parser.add_argument("--config", type=Path,
                        default=root / "scripts/bag_lio_mapping/kiss_icp_config.yaml")
    parser.add_argument("--csv", type=Path,
                        default=root / "pcds/bag_only_scan_alignment.csv")
    parser.add_argument("--diagnostics", type=Path,
                        default=root / "pcds/bag_only_scan_alignment.json")
    parser.add_argument("--maximum-horizontal-distance", type=float, default=0.45)
    parser.add_argument("--datum-scans", type=int, default=20)
    parser.add_argument("--point-stride", type=int, default=4)
    parser.add_argument("--odometry-topic", default="/mavros/local_position/odom")
    parser.add_argument("--gnss-topic", default="/mavros/global_position/global")
    args = parser.parse_args()

    pose_data = np.load(args.poses)
    poses, pose_times = pose_data["poses"], pose_data["timestamps"]
    odom_times, odom_poses = odometry_trajectory(args.bag, args.odometry_topic)
    map_from_local = interpolate_odometry(odom_times, odom_poses, float(pose_times[0]))
    horizontal_rotation, horizontal_translation, horizontal_rmse, epsg = \
        horizontal_gnss_registration(args.bag, odom_times, odom_poses, args.gnss_topic)

    reference = load_ascii_pcd_xyz(args.reference)
    reference_tree = cKDTree(reference[:, :2])
    settings = yaml.safe_load(args.config.read_text())
    extrinsic = settings["extrinsic_base_from_lidar"]
    lidar_rotation = quaternion_matrix(extrinsic["quaternion_xyzw"])
    lidar_translation = np.asarray(extrinsic["translation_m"], dtype=np.float64)

    rows, previous, scan = [], poses[0], 0
    with AnyReader([args.bag]) as reader:
        connections = [item for item in reader.connections if item.topic == "/velodyne_points"]
        for _, timestamp, raw in reader.messages(connections=connections):
            if scan >= len(poses):
                break
            message = reader.deserialize(raw, connections[0].msgtype)
            points, relative_times = valid_body_scan(
                message, lidar_rotation, lidar_translation, 1.0, 80.0,
            )
            points, relative_times = points[::args.point_stride], relative_times[::args.point_stride]
            local = transform_timed_points(points, relative_times, previous, poses[scan])
            odom_frame = (map_from_local[:3, :3] @ local.T).T + map_from_local[:3, 3]
            global_xy = odom_frame[:, :2] @ horizontal_rotation + horizontal_translation
            distance, indices = reference_tree.query(global_xy, workers=-1)
            residual = odom_frame[:, 2] - reference[indices, 2]
            bias, spread, count = robust_ground_bias(
                residual[distance < args.maximum_horizontal_distance],
            )
            rows.append((timestamp * 1e-9, bias, spread, count))
            previous, scan = poses[scan], scan + 1
            if scan % 500 == 0:
                print(f"Evaluated {scan} scans", flush=True)

    values = np.asarray(rows, dtype=np.float64)
    datum = float(np.nanmedian(values[:args.datum_scans, 1]))
    values[:, 1] -= datum
    np.savetxt(args.csv, values, delimiter=",",
               header="time_s,bias_m,nmad_m,count", comments="")
    bias = values[:, 1]
    finite = np.isfinite(bias)
    median = float(np.nanmedian(bias))
    result = {
        "method": "scan-indexed held-out Bayern ground evaluation",
        "reference_used_for_estimation": False,
        "poses": str(args.poses),
        "reference": str(args.reference),
        "evaluated_scans": int(np.count_nonzero(finite)),
        "horizontal_registration": {
            "source": "onboard GNSS only", "epsg": epsg, "rmse_m": horizontal_rmse,
        },
        "vertical_datum": {
            "source": f"held-out first {args.datum_scans} grounded scans", "offset_m": datum,
        },
        "per_scan_vertical_bias_m": {
            "mean": float(np.nanmean(bias)), "median": median,
            "rmse": float(np.sqrt(np.nanmean(bias ** 2))),
            "nmad": float(1.4826 * np.nanmedian(np.abs(bias - median))),
            "p95_absolute": float(np.nanpercentile(np.abs(bias), 95)),
        },
        "median_within_scan_ground_nmad_m": float(np.nanmedian(values[:, 2])),
        "maximum_horizontal_correspondence_m": args.maximum_horizontal_distance,
    }
    args.diagnostics.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
