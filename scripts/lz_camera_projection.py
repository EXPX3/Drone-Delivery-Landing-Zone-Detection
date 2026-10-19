#!/usr/bin/env python3
"""Geometry and calibration helpers for landing-zone camera projection."""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import cv2
import numpy as np


@dataclass(frozen=True)
class Transform:
    """Rigid transform whose matrix maps child-frame points into the parent."""

    translation: np.ndarray
    quaternion_xyzw: np.ndarray

    @staticmethod
    def identity() -> "Transform":
        return Transform(np.zeros(3), np.array([0.0, 0.0, 0.0, 1.0]))

    @property
    def matrix(self) -> np.ndarray:
        x, y, z, w = normalized_quaternion(self.quaternion_xyzw)
        rotation = np.array(
            [
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
            ],
            dtype=np.float64,
        )
        result = np.eye(4, dtype=np.float64)
        result[:3, :3] = rotation
        result[:3, 3] = self.translation
        return result

    def inverse(self) -> "Transform":
        matrix = np.linalg.inv(self.matrix)
        return transform_from_matrix(matrix)


def normalized_quaternion(quaternion: Iterable[float]) -> np.ndarray:
    value = np.asarray(quaternion, dtype=np.float64)
    norm = np.linalg.norm(value)
    if norm < 1e-12:
        raise ValueError("Quaternion has zero length")
    return value / norm


def transform_from_matrix(matrix: np.ndarray) -> Transform:
    rotation = np.asarray(matrix[:3, :3], dtype=np.float64)
    trace = np.trace(rotation)
    if trace > 0.0:
        scale = np.sqrt(trace + 1.0) * 2.0
        quaternion = np.array(
            [
                (rotation[2, 1] - rotation[1, 2]) / scale,
                (rotation[0, 2] - rotation[2, 0]) / scale,
                (rotation[1, 0] - rotation[0, 1]) / scale,
                0.25 * scale,
            ]
        )
    else:
        index = int(np.argmax(np.diag(rotation)))
        if index == 0:
            scale = np.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
            quaternion = np.array(
                [0.25 * scale, (rotation[0, 1] + rotation[1, 0]) / scale,
                 (rotation[0, 2] + rotation[2, 0]) / scale,
                 (rotation[2, 1] - rotation[1, 2]) / scale]
            )
        elif index == 1:
            scale = np.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
            quaternion = np.array(
                [(rotation[0, 1] + rotation[1, 0]) / scale, 0.25 * scale,
                 (rotation[1, 2] + rotation[2, 1]) / scale,
                 (rotation[0, 2] - rotation[2, 0]) / scale]
            )
        else:
            scale = np.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
            quaternion = np.array(
                [(rotation[0, 2] + rotation[2, 0]) / scale,
                 (rotation[1, 2] + rotation[2, 1]) / scale, 0.25 * scale,
                 (rotation[1, 0] - rotation[0, 1]) / scale]
            )
    return Transform(np.asarray(matrix[:3, 3], dtype=np.float64), normalized_quaternion(quaternion))


def compose(*transforms: Transform) -> Transform:
    matrix = np.eye(4, dtype=np.float64)
    for transform in transforms:
        matrix = matrix @ transform.matrix
    return transform_from_matrix(matrix)


def interpolate_transform(first: Transform, second: Transform, fraction: float) -> Transform:
    fraction = float(np.clip(fraction, 0.0, 1.0))
    translation = (1.0 - fraction) * first.translation + fraction * second.translation
    q0 = normalized_quaternion(first.quaternion_xyzw)
    q1 = normalized_quaternion(second.quaternion_xyzw)
    if np.dot(q0, q1) < 0.0:
        q1 = -q1
    quaternion = normalized_quaternion((1.0 - fraction) * q0 + fraction * q1)
    return Transform(translation, quaternion)


class TransformSeries:
    def __init__(self, samples: list[tuple[int, Transform]], max_gap_s: float = 0.25):
        if not samples:
            raise ValueError("Transform series is empty")
        ordered = sorted(samples, key=lambda item: item[0])
        self.times = np.asarray([item[0] for item in ordered], dtype=np.int64)
        self.transforms = [item[1] for item in ordered]
        self.max_gap_ns = int(max_gap_s * 1e9)

    def at(self, timestamp_ns: int) -> Transform:
        right = int(np.searchsorted(self.times, timestamp_ns, side="left"))
        if right == 0:
            if abs(int(self.times[0]) - timestamp_ns) > self.max_gap_ns:
                raise LookupError("Transform requested before available samples")
            return self.transforms[0]
        if right == len(self.times):
            if timestamp_ns - int(self.times[-1]) > self.max_gap_ns:
                raise LookupError("Transform requested after available samples")
            return self.transforms[-1]
        left = right - 1
        interval = int(self.times[right] - self.times[left])
        if interval > self.max_gap_ns:
            raise LookupError(f"Transform gap is {interval / 1e9:.3f} s")
        fraction = (timestamp_ns - int(self.times[left])) / interval if interval else 0.0
        return interpolate_transform(self.transforms[left], self.transforms[right], fraction)


def latlon_to_utm32(latitude_deg: float, longitude_deg: float) -> tuple[float, float]:
    """Convert WGS84 latitude/longitude to ETRS89/UTM 32N-scale coordinates.

    WGS84 and ETRS89 differ by only decimetres at this site; final registration is
    fitted to local odometry, so the closed-form UTM conversion avoids a runtime
    dependency on PROJ inside the ROS bag container.
    """

    a = 6378137.0
    eccentricity_sq = 6.69437999014e-3
    k0 = 0.9996
    latitude = np.deg2rad(latitude_deg)
    longitude = np.deg2rad(longitude_deg)
    central_meridian = np.deg2rad(9.0)
    ep_sq = eccentricity_sq / (1.0 - eccentricity_sq)
    n = a / np.sqrt(1.0 - eccentricity_sq * np.sin(latitude) ** 2)
    t = np.tan(latitude) ** 2
    c = ep_sq * np.cos(latitude) ** 2
    aa = np.cos(latitude) * (longitude - central_meridian)
    m = a * (
        (1 - eccentricity_sq / 4 - 3 * eccentricity_sq**2 / 64 - 5 * eccentricity_sq**3 / 256) * latitude
        - (3 * eccentricity_sq / 8 + 3 * eccentricity_sq**2 / 32 + 45 * eccentricity_sq**3 / 1024)
        * np.sin(2 * latitude)
        + (15 * eccentricity_sq**2 / 256 + 45 * eccentricity_sq**3 / 1024) * np.sin(4 * latitude)
        - (35 * eccentricity_sq**3 / 3072) * np.sin(6 * latitude)
    )
    easting = k0 * n * (
        aa + (1 - t + c) * aa**3 / 6 + (5 - 18 * t + t**2 + 72 * c - 58 * ep_sq) * aa**5 / 120
    ) + 500000.0
    northing = k0 * (
        m + n * np.tan(latitude) * (
            aa**2 / 2 + (5 - t + 9 * c + 4 * c**2) * aa**4 / 24
            + (61 - 58 * t + t**2 + 600 * c - 330 * ep_sq) * aa**6 / 720
        )
    )
    return float(easting), float(northing)


def fit_planar_rigid(source_xy: np.ndarray, target_xy: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Robustly fit target = R @ source + translation and return residuals."""

    source = np.asarray(source_xy, dtype=np.float64)
    target = np.asarray(target_xy, dtype=np.float64)
    if source.shape != target.shape or source.ndim != 2 or source.shape[1] != 2 or len(source) < 3:
        raise ValueError("At least three paired 2D points are required")
    keep = np.ones(len(source), dtype=bool)
    for _ in range(4):
        src = source[keep]
        dst = target[keep]
        src_center = np.mean(src, axis=0)
        dst_center = np.mean(dst, axis=0)
        u, _, vt = np.linalg.svd((src - src_center).T @ (dst - dst_center))
        rotation = vt.T @ u.T
        if np.linalg.det(rotation) < 0:
            vt[-1] *= -1
            rotation = vt.T @ u.T
        translation = dst_center - rotation @ src_center
        residuals = np.linalg.norm((rotation @ source.T).T + translation - target, axis=1)
        median = np.median(residuals[keep])
        mad = np.median(np.abs(residuals[keep] - median))
        threshold = max(0.35, median + 3.5 * 1.4826 * mad)
        new_keep = residuals <= threshold
        if np.array_equal(new_keep, keep) or np.count_nonzero(new_keep) < 3:
            break
        keep = new_keep
    return rotation, translation, residuals


def scaled_camera_matrix(native_matrix: np.ndarray, native_size: tuple[int, int], image_size: tuple[int, int]) -> np.ndarray:
    sx = image_size[0] / native_size[0]
    sy = image_size[1] / native_size[1]
    if abs(sx - sy) > 1e-6:
        raise ValueError("Full-frame calibration scaling requires an unchanged aspect ratio")
    matrix = np.asarray(native_matrix, dtype=np.float64).copy()
    matrix[0, :] *= sx
    matrix[1, :] *= sy
    return matrix


def load_landing_zones(path: Path, categories: set[str] | None, max_zones: int) -> list[dict[str, float | int | str]]:
    zones: list[dict[str, float | int | str]] = []
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            category = row.get("risk_category", "safest")
            if categories and category not in categories:
                continue
            rank_value = row["rank_risk"] if "rank_risk" in row else row["rank"]
            zones.append(
                {
                    "rank": int(rank_value),
                    "category": category,
                    "x": float(row["x"]),
                    "y": float(row["y"]),
                    "z": float(row["z"]),
                    "radius": float(row["radius"]),
                    "normal_x": float(row.get("normal_x", 0.0)),
                    "normal_y": float(row.get("normal_y", 0.0)),
                    "normal_z": float(row.get("normal_z", 1.0)),
                }
            )
            if max_zones > 0 and len(zones) >= max_zones:
                break
    if not zones:
        raise ValueError(f"No landing zones selected from {path}")
    return zones


def zone_ring(zone: dict[str, float | int | str], vertical_offset_m: float, samples: int = 72) -> np.ndarray:
    angles = np.linspace(0.0, 2.0 * np.pi, samples, endpoint=False)
    center = np.array(
        [float(zone["x"]), float(zone["y"]), float(zone["z"]) + vertical_offset_m],
        dtype=np.float64,
    )
    normal = np.array(
        [float(zone.get("normal_x", 0.0)), float(zone.get("normal_y", 0.0)),
         float(zone.get("normal_z", 1.0))],
        dtype=np.float64,
    )
    normal_length = np.linalg.norm(normal)
    if normal_length < 1e-9:
        normal = np.array([0.0, 0.0, 1.0])
    else:
        normal /= normal_length
    reference = np.array([0.0, 0.0, 1.0]) if abs(normal[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    axis_u = np.cross(normal, reference)
    axis_u /= np.linalg.norm(axis_u)
    axis_v = np.cross(normal, axis_u)
    return center + float(zone["radius"]) * (
        np.cos(angles)[:, None] * axis_u + np.sin(angles)[:, None] * axis_v
    )


def transform_points(transform: Transform, points: np.ndarray) -> np.ndarray:
    homogeneous = np.column_stack((np.asarray(points, dtype=np.float64), np.ones(len(points))))
    return (transform.matrix @ homogeneous.T).T[:, :3]


def project_optical_points(points: np.ndarray, camera_matrix: np.ndarray, distortion: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    points = np.asarray(points, dtype=np.float64)
    visible = points[:, 2] > 0.1
    pixels = np.full((len(points), 2), np.nan, dtype=np.float64)
    if np.any(visible):
        projected, _ = cv2.projectPoints(
            points[visible], np.zeros(3), np.zeros(3), camera_matrix, distortion
        )
        pixels[visible] = projected.reshape(-1, 2)
    return pixels, visible

