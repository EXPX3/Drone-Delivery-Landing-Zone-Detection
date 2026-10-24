#!/usr/bin/env python3
"""Replay LiDAR scans through a frozen, validated bag-only pose trajectory."""

from __future__ import annotations

from pathlib import Path

import numpy as np


def quaternion_matrix(quaternion_xyzw: list[float]) -> np.ndarray:
    x, y, z, w = np.asarray(quaternion_xyzw, dtype=np.float64)
    norm = x * x + y * y + z * z + w * w
    if norm < 1e-12:
        raise ValueError("LiDAR extrinsic quaternion has zero length")
    scale = 2.0 / norm
    return np.array([
        [1.0 - scale * (y * y + z * z), scale * (x * y - z * w), scale * (x * z + y * w)],
        [scale * (x * y + z * w), 1.0 - scale * (x * x + z * z), scale * (y * z - x * w)],
        [scale * (x * z - y * w), scale * (y * z + x * w), 1.0 - scale * (x * x + y * y)],
    ])


def timed_xyz(message: object) -> tuple[np.ndarray, np.ndarray]:
    """Decode XYZ and relative firing time with one shared validity mask."""
    fields = {item.name: item for item in message.fields}
    if not {"x", "y", "z", "time"}.issubset(fields):
        raise ValueError("PointCloud2 requires x, y, z and time fields for corrected replay")
    formats = {7: "<f4", 8: "<f8"}
    try:
        dtype = np.dtype({
            "names": ["x", "y", "z", "time"],
            "formats": [formats[int(fields[name].datatype)] for name in ("x", "y", "z", "time")],
            "offsets": [int(fields[name].offset) for name in ("x", "y", "z", "time")],
            "itemsize": int(message.point_step),
        })
    except KeyError as error:
        raise ValueError(f"Unsupported PointCloud2 floating-point datatype: {error.args[0]}") from error
    count = int(message.width) * int(message.height)
    values = np.frombuffer(message.data, dtype=dtype, count=count)
    points = np.column_stack((values["x"], values["y"], values["z"])).astype(np.float64)
    relative_times = np.asarray(values["time"], dtype=np.float64)
    valid = np.isfinite(points).all(axis=1) & np.isfinite(relative_times)
    return points[valid], relative_times[valid]


def rotation_axis_angle(rotation: np.ndarray) -> tuple[np.ndarray, float]:
    angle = float(np.arccos(np.clip((np.trace(rotation) - 1.0) * 0.5, -1.0, 1.0)))
    if angle < 1e-12:
        return np.array([1.0, 0.0, 0.0]), 0.0
    axis = np.array([
        rotation[2, 1] - rotation[1, 2],
        rotation[0, 2] - rotation[2, 0],
        rotation[1, 0] - rotation[0, 1],
    ]) / (2.0 * np.sin(angle))
    return axis, angle


def transform_timed_points(points: np.ndarray, relative_times: np.ndarray,
                           previous: np.ndarray, current: np.ndarray) -> np.ndarray:
    """Interpolate translation and SO(3) rotation across one rotating LiDAR scan."""
    minimum, maximum = float(np.min(relative_times)), float(np.max(relative_times))
    phase = (relative_times - minimum) / max(maximum - minimum, 1e-12)
    position = previous[:3, 3] * (1.0 - phase[:, None]) + current[:3, 3] * phase[:, None]
    relative_rotation = previous[:3, :3].T @ current[:3, :3]
    axis, angle = rotation_axis_angle(relative_rotation)
    theta = phase * angle
    cross = np.cross(np.broadcast_to(axis, points.shape), points)
    dot = points @ axis
    rotated = (
        points * np.cos(theta)[:, None]
        + cross * np.sin(theta)[:, None]
        + axis[None, :] * dot[:, None] * (1.0 - np.cos(theta))[:, None]
    )
    return (previous[:3, :3] @ rotated.T).T + position


class ValidatedPoseReplay:
    """Timestamp-check and deskew scans with the frozen strip-optimized poses."""

    def __init__(self, pose_path: Path, extrinsic: dict, maximum_time_error_s: float = 0.03) -> None:
        data = np.load(pose_path)
        self.poses = np.asarray(data["poses"], dtype=np.float64)
        self.timestamps = np.asarray(data["timestamps"], dtype=np.float64)
        if self.poses.shape != (len(self.timestamps), 4, 4) or len(self.poses) < 2:
            raise ValueError(f"Invalid optimized pose archive: {pose_path}")
        if np.any(np.diff(self.timestamps) <= 0.0):
            raise ValueError("Optimized pose timestamps must be strictly increasing")
        self.maximum_time_error_s = float(maximum_time_error_s)
        self.rotation = quaternion_matrix(extrinsic["quaternion_xyzw"])
        self.translation = np.asarray(extrinsic["translation_m"], dtype=np.float64)
        self.next_index = 0
        self.world_from_local: np.ndarray | None = None
        self.maximum_observed_time_error_s = 0.0

    def initialize_world_alignment(self, world_from_body: np.ndarray) -> None:
        self.world_from_local = np.asarray(world_from_body, dtype=np.float64) @ np.linalg.inv(self.poses[0])

    def pose_index(self, timestamp_ns: int) -> tuple[int, float]:
        timestamp = timestamp_ns * 1e-9
        right = int(np.searchsorted(self.timestamps, timestamp, side="left"))
        candidates = [index for index in (right - 1, right) if 0 <= index < len(self.timestamps)]
        index = min(candidates, key=lambda value: abs(self.timestamps[value] - timestamp))
        error = abs(float(self.timestamps[index] - timestamp))
        if error > self.maximum_time_error_s:
            raise RuntimeError(
                f"LiDAR/optimized-pose timestamp mismatch {error:.4f} s exceeds "
                f"{self.maximum_time_error_s:.4f} s"
            )
        if index < self.next_index:
            raise RuntimeError(f"Optimized pose index moved backward from {self.next_index} to {index}")
        self.next_index = index + 1
        self.maximum_observed_time_error_s = max(self.maximum_observed_time_error_s, error)
        return index, error

    def transform(self, message: object, timestamp_ns: int, stride: int,
                  minimum_range_m: float, maximum_range_m: float) -> tuple[np.ndarray, int, float]:
        if self.world_from_local is None:
            raise RuntimeError("World alignment must be initialized before transforming scans")
        index, time_error = self.pose_index(timestamp_ns)
        lidar, relative_times = timed_xyz(message)
        ranges = np.linalg.norm(lidar, axis=1)
        keep = (ranges >= minimum_range_m) & (ranges <= maximum_range_m)
        lidar, relative_times = lidar[keep][::stride], relative_times[keep][::stride]
        body = (self.rotation @ lidar.T).T + self.translation
        previous = self.poses[max(0, index - 1)]
        local = transform_timed_points(body, relative_times, previous, self.poses[index])
        world = (self.world_from_local[:3, :3] @ local.T).T + self.world_from_local[:3, 3]
        return world, index, time_error

    def sensor_origin(self, index: int) -> np.ndarray:
        if self.world_from_local is None:
            raise RuntimeError("World alignment must be initialized before requesting sensor origin")
        position = self.poses[index, :3, 3]
        return self.world_from_local[:3, :3] @ position + self.world_from_local[:3, 3]
