#!/usr/bin/env python3
"""Build an update-only LiDAR map and quantify ground layering and motion error."""

from __future__ import annotations

import argparse
import csv
import json
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np
import yaml
from scipy.signal import welch
from scipy.spatial import cKDTree

SCRIPTS = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPTS))

from lz_bag_replay_viewer.replay_server import orthometric_from_map  # noqa: E402
from lz_camera_projection import transform_points  # noqa: E402
from lz_rviz_viewer.dynamic_occupancy import apply_centered_transform, pointcloud2_xyz  # noqa: E402
from lz_rviz_viewer.replay_node import read_ascii_pcd  # noqa: E402
from project_lz_into_bag_camera import (  # noqa: E402
    chain_transform, config_transform, fit_utm_to_map, open_reader, read_pose_and_gps,
    require_ros, stamp_ns,
)


class Reservoir:
    def __init__(self, capacity: int, seed: int = 23) -> None:
        self.capacity = capacity
        self.values = np.empty(capacity, dtype=np.float64)
        self.count = 0
        self.rng = np.random.default_rng(seed)

    def add(self, values: np.ndarray) -> None:
        for value in np.asarray(values, dtype=np.float64):
            self.count += 1
            if self.count <= self.capacity:
                self.values[self.count - 1] = value
            else:
                index = int(self.rng.integers(self.count))
                if index < self.capacity:
                    self.values[index] = value

    def array(self) -> np.ndarray:
        return self.values[:min(self.count, self.capacity)]


def filtered_reader(path: Path, topics: list[str]):
    rosbag2_py, _, _ = require_ros()
    reader = open_reader(path)
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    return reader


def read_imu(path: Path, topic: str) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    _, deserialize, get_message = require_ros()
    reader = filtered_reader(path, [topic])
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    message_type = get_message(types[topic])
    times, angular, acceleration = [], [], []
    while reader.has_next():
        name, raw, bag_time = reader.read_next()
        message = deserialize(raw, message_type)
        times.append(stamp_ns(message, bag_time))
        angular.append([message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z])
        acceleration.append([message.linear_acceleration.x, message.linear_acceleration.y,
                             message.linear_acceleration.z])
    return np.asarray(times, dtype=np.int64), np.asarray(angular), np.asarray(acceleration)


def nearest_rows(times: np.ndarray, query: np.ndarray) -> np.ndarray:
    right = np.searchsorted(times, query)
    right = np.clip(right, 0, len(times) - 1)
    left = np.maximum(0, right - 1)
    choose_left = np.abs(query - times[left]) <= np.abs(times[right] - query)
    return np.where(choose_left, left, right)


def write_pcd(path: Path, points: np.ndarray) -> None:
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z count std_z\n"
        "SIZE 4 4 4 4 4\nTYPE F F F F F\nCOUNT 1 1 1 1 1\n"
        f"WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(points)}\nDATA ascii\n"
    )
    with path.open("w", encoding="ascii") as handle:
        handle.write(header)
        np.savetxt(handle, points, fmt="%.6f %.6f %.6f %.0f %.6f")


def spectral_peaks(values: np.ndarray, times: np.ndarray) -> dict:
    if len(values) < 16:
        return {"sample_rate_hz": 0.0, "peaks_hz": []}
    dt = np.diff(times) / 1e9
    sample_rate = 1.0 / float(np.median(dt[dt > 0]))
    magnitude = np.linalg.norm(values - np.median(values, axis=0), axis=1)
    frequency, power = welch(magnitude, fs=sample_rate, nperseg=min(1024, len(magnitude)))
    keep = frequency >= 0.5
    indices = np.flatnonzero(keep)[np.argsort(power[keep])[-5:][::-1]]
    return {"sample_rate_hz": sample_rate, "nyquist_hz": sample_rate / 2.0,
            "peaks_hz": [float(frequency[index]) for index in indices],
            "peak_power": [float(power[index]) for index in indices]}


def robust_metrics(values: np.ndarray) -> dict:
    median = float(np.median(values))
    return {
        "count": int(len(values)), "mean_m": float(np.mean(values)), "median_m": median,
        "rmse_m": float(np.sqrt(np.mean(values ** 2))),
        "nmad_m": float(1.4826 * np.median(np.abs(values - median))),
        "p95_absolute_m": float(np.percentile(np.abs(values), 95)),
        "p99_absolute_m": float(np.percentile(np.abs(values), 99)),
    }


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--config", type=Path, default=root / "scripts/config/lz_camera_projection.yaml")
    parser.add_argument("--ground-pcd", type=Path, default=root / "pcds/696_5328_cropped_ground_only.pcd")
    parser.add_argument("--full-pcd", type=Path, default=root / "pcds/696_5328_cropped_by_bayernatlas_20260812084517.pcd")
    parser.add_argument("--output", type=Path, default=root / "pcds/lidar_mapping_accuracy")
    parser.add_argument("--cell-size", type=float, default=0.20)
    parser.add_argument("--point-stride", type=int, default=4)
    parser.add_argument("--maximum-range", type=float, default=80.0)
    parser.add_argument("--ground-gate", type=float, default=1.5)
    parser.add_argument("--max-scans", type=int, default=0)
    parser.add_argument("--disable-lidar-correction", action="store_true")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)

    config = yaml.safe_load(args.config.read_text())
    tf_data, gps = read_pose_and_gps(args.bag, config)
    utm_to_map, registration = fit_utm_to_map(tf_data, gps, config)
    vertical_offset = float(config["registration"]["lz_to_gnss_vertical_offset_m"])
    map_to_ortho = orthometric_from_map(utm_to_map, vertical_offset)
    correction = config_transform(config["registration"]["live_lidar_to_static"])
    if args.disable_lidar_correction:
        correction = type(correction)(np.zeros(3), np.array([0.0, 0.0, 0.0, 1.0]))
    full_cloud = read_ascii_pcd(args.full_pcd, 0)
    pivot = np.median(full_cloud, axis=0)
    full_tree = cKDTree(full_cloud)
    ground = read_ascii_pcd(args.ground_pcd, 0)
    ground_tree = cKDTree(ground[:, :2])

    imu_topic = "/mavros/imu/data_raw"
    imu_times, angular, acceleration = read_imu(args.bag, imu_topic)
    _, deserialize, get_message = require_ros()
    lidar_topic = config["topics"]["lidar_points"]
    reader = filtered_reader(args.bag, [lidar_topic])
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    lidar_type = get_message(types[lidar_topic])

    cells: dict[tuple[int, int], list[float]] = {}
    obstacle_cells: dict[tuple[int, int, int], list[float]] = {}
    raw_residual_sample = Reservoir(1_000_000)
    corrected_residual_sample = Reservoir(1_000_000, seed=29)
    scan_rows = []
    total_points = ground_points = changed_cells = scans = 0
    while reader.has_next() and (not args.max_scans or scans < args.max_scans):
        _, raw, bag_time = reader.read_next()
        message = deserialize(raw, lidar_type)
        timestamp = stamp_ns(message, bag_time)
        sensor = pointcloud2_xyz(message)[::args.point_stride]
        ranges = np.linalg.norm(sensor, axis=1)
        sensor = sensor[(ranges > 1.0) & (ranges <= args.maximum_range)]
        total_points += len(sensor)
        try:
            map_to_lidar = chain_transform(tf_data, config["frames"]["lidar_chain"], timestamp,
                                           float(config["synchronization"]["max_tf_gap_s"]))
        except LookupError:
            continue
        mapped = transform_points(map_to_ortho, transform_points(map_to_lidar, sensor))
        mapped = apply_centered_transform(mapped, correction.matrix, pivot)
        distance, index = ground_tree.query(mapped[:, :2], workers=-1)
        residual = mapped[:, 2] - ground[index, 2]
        keep = (distance <= 0.45) & (np.abs(residual) <= args.ground_gate)
        points, residual = mapped[keep], residual[keep]
        if len(points) < 20:
            continue
        raw_residual_sample.add(residual)
        vertical_bias = float(np.median(residual))
        mapped[:, 2] -= vertical_bias
        corrected_residual = mapped[:, 2] - ground[index, 2]
        corrected_ground = (distance <= 0.45) & (np.abs(corrected_residual) <= 0.30)
        points = mapped[corrected_ground]
        residual = corrected_residual[corrected_ground]
        ground_points += len(points)
        corrected_residual_sample.add(residual)

        origin = apply_centered_transform(
            transform_points(map_to_ortho, map_to_lidar.translation[None, :]),
            correction.matrix, pivot,
        )[0]
        relative = points[:, :2] - origin[:2]
        design = np.column_stack((relative, np.ones(len(relative))))
        coefficient, _, _, _ = np.linalg.lstsq(design, residual, rcond=None)
        tilt_x, tilt_y = np.rad2deg(np.arctan(coefficient[:2]))
        scan_rows.append([timestamp, len(points), vertical_bias, float(np.median(residual)),
                          float(np.sqrt(np.mean(residual ** 2))), tilt_x, tilt_y])

        changed = (distance <= 0.45) & (corrected_residual >= 0.30) & (corrected_residual <= 5.0)
        changed_points = mapped[changed]
        if len(changed_points):
            static_distance, _ = full_tree.query(changed_points, workers=-1)
            changed_points = changed_points[static_distance >= 0.30]
        obstacle_keys = np.floor(changed_points / args.cell_size).astype(np.int64)
        for key in np.unique(obstacle_keys, axis=0):
            mask = np.all(obstacle_keys == key, axis=1)
            center = np.median(changed_points[mask], axis=0)
            item = obstacle_cells.get(tuple(int(value) for value in key))
            if item is None:
                obstacle_cells[tuple(int(value) for value in key)] = [1.0, *center]
            else:
                item[0] += 1.0
                item[1:] = np.asarray(item[1:]) + (center - np.asarray(item[1:])) / item[0]

        keys = np.floor(points[:, :2] / args.cell_size).astype(np.int64)
        for key in np.unique(keys, axis=0):
            mask = np.all(keys == key, axis=1)
            z = float(np.median(points[mask, 2]))
            item = cells.get((int(key[0]), int(key[1])))
            if item is None:
                cells[(int(key[0]), int(key[1]))] = [1.0, z, 0.0, z, z]
                changed_cells += 1
            else:
                count, mean, m2, minimum, maximum = item
                count += 1.0
                delta = z - mean
                mean += delta / count
                m2 += delta * (z - mean)
                item[:] = [count, mean, m2, min(minimum, z), max(maximum, z)]
                if abs(delta) >= 0.01:
                    changed_cells += 1
        scans += 1
        if scans % 250 == 0:
            print(f"Processed {scans} scans, {ground_points} ground returns, {len(cells)} cells", flush=True)

    scan_data = np.asarray(scan_rows, dtype=np.float64)
    imu_index = nearest_rows(imu_times, scan_data[:, 0].astype(np.int64))
    angular_speed = np.linalg.norm(angular[imu_index], axis=1)
    acceleration_dynamic = np.linalg.norm(
        acceleration[imu_index] - np.median(acceleration, axis=0), axis=1,
    )
    median_residual = scan_data[:, 2]
    tilt_magnitude = np.linalg.norm(scan_data[:, 5:7], axis=1)
    acceleration_dynamic = np.abs(
        np.linalg.norm(acceleration[imu_index], axis=1) - np.median(np.linalg.norm(acceleration, axis=1))
    )
    correlation = {
        "residual_vs_angular_speed": float(np.corrcoef(median_residual, angular_speed)[0, 1]),
        "absolute_residual_vs_angular_speed": float(np.corrcoef(np.abs(median_residual), angular_speed)[0, 1]),
        "tilt_vs_angular_speed": float(np.corrcoef(tilt_magnitude, angular_speed)[0, 1]),
        "residual_vs_dynamic_acceleration": float(np.corrcoef(median_residual, acceleration_dynamic)[0, 1]),
        "tilt_vs_dynamic_acceleration": float(np.corrcoef(tilt_magnitude, acceleration_dynamic)[0, 1]),
    }

    map_rows, repeated_std, repeated_range = [], [], []
    for (ix, iy), (count, mean, m2, minimum, maximum) in cells.items():
        std = np.sqrt(m2 / (count - 1.0)) if count > 1 else 0.0
        map_rows.append([(ix + 0.5) * args.cell_size, (iy + 0.5) * args.cell_size,
                         mean, count, std])
        if count >= 3:
            repeated_std.append(std)
            repeated_range.append(maximum - minimum)
    map_rows = np.asarray(map_rows)
    write_pcd(args.output / "update_only_ground_map.pcd", map_rows)
    obstacle_rows = np.asarray([
        [values[1], values[2], values[3], values[0], 0.0]
        for values in obstacle_cells.values() if values[0] >= 3
    ], dtype=np.float64)
    write_pcd(args.output / "change_only_obstacles.pcd", obstacle_rows)
    with (args.output / "per_scan_metrics.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["timestamp_ns", "ground_points", "raw_vertical_bias_m",
                         "corrected_median_residual_m", "corrected_rmse_m",
                         "apparent_tilt_x_deg", "apparent_tilt_y_deg", "angular_speed_rad_s",
                         "dynamic_acceleration_m_s2"])
        for row, gyro, accel in zip(scan_data, angular_speed, acceleration_dynamic):
            writer.writerow([*row, gyro, accel])

    repeated_std = np.asarray(repeated_std)
    repeated_range = np.asarray(repeated_range)
    report = {
        "method": {
            "cell_size_m": args.cell_size, "point_stride": args.point_stride,
            "lidar_correction_enabled": not args.disable_lidar_correction,
            "ground_reference": str(args.ground_pcd), "ground_gate_m": args.ground_gate,
            "changed_cell_updates": changed_cells,
            "note": "Output map stores one final point per XY cell; repeated samples are retained only as statistics.",
        },
        "registration": registration,
        "counts": {"scans": scans, "input_points": total_points, "ground_returns": ground_points,
                   "unique_ground_cells": len(cells), "repeated_ground_cells": len(repeated_std),
                   "confirmed_change_voxels": len(obstacle_rows)},
        "ground_accuracy_before_vertical_registration": robust_metrics(raw_residual_sample.array()),
        "ground_accuracy_after_vertical_registration": robust_metrics(corrected_residual_sample.array()),
        "layering": {
            "median_repeated_std_m": float(np.median(repeated_std)),
            "p95_repeated_std_m": float(np.percentile(repeated_std, 95)),
            "median_peak_to_peak_m": float(np.median(repeated_range)),
            "p95_peak_to_peak_m": float(np.percentile(repeated_range, 95)),
            "maximum_peak_to_peak_m": float(np.max(repeated_range)),
        },
        "apparent_scan_tilt_deg": {
            "median": float(np.median(tilt_magnitude)), "p95": float(np.percentile(tilt_magnitude, 95)),
            "maximum": float(np.max(tilt_magnitude)),
        },
        "motion_correlation_pearson": correlation,
        "imu_angular_spectrum": spectral_peaks(angular, imu_times),
        "imu_acceleration_spectrum": spectral_peaks(acceleration, imu_times),
        "limitations": [
            "The 35 Hz bag IMU cannot resolve vibration above its Nyquist frequency.",
            "PointCloud2 scans have one header timestamp; this audit cannot deskew individual laser firing times.",
            "Bayern ground sampling and vegetation changes contribute to residuals independently of the mount.",
        ],
    }
    (args.output / "mapping_accuracy_report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
