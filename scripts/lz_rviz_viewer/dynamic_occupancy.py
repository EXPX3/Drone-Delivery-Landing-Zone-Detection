#!/usr/bin/env python3
"""Sparse live occupancy and conservative landing-zone revalidation."""

from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass, field

import numpy as np


def interpolate_gps(gps: list[tuple[int, float, float, float]], timestamp_ns: int,
                    vertical_offset: float, times: np.ndarray | None = None) -> np.ndarray:
    """Interpolate UTM/GNSS position at an image timestamp in orthometric Z."""
    if times is None:
        times = np.fromiter((item[0] for item in gps), dtype=np.int64)
    right = int(np.searchsorted(times, timestamp_ns, side="left"))
    if right <= 0:
        item = gps[0]
        return np.array([item[1], item[2], item[3] - vertical_offset])
    if right >= len(gps):
        item = gps[-1]
        return np.array([item[1], item[2], item[3] - vertical_offset])
    first, second = gps[right - 1], gps[right]
    fraction = (timestamp_ns - first[0]) / max(1, second[0] - first[0])
    first_position = np.array([first[1], first[2], first[3] - vertical_offset])
    second_position = np.array([second[1], second[2], second[3] - vertical_offset])
    return first_position * (1.0 - fraction) + second_position * fraction


def pointcloud2_xyz(message: object) -> np.ndarray:
    """Decode x/y/z float32 fields from an organized or flat PointCloud2."""
    offsets = {item.name: int(item.offset) for item in message.fields}
    if not {"x", "y", "z"}.issubset(offsets):
        raise ValueError("PointCloud2 does not contain x, y, z fields")
    count = int(message.width) * int(message.height)
    dtype = np.dtype({
        "names": ["x", "y", "z"],
        "formats": ["<f4", "<f4", "<f4"],
        "offsets": [offsets["x"], offsets["y"], offsets["z"]],
        "itemsize": int(message.point_step),
    })
    values = np.frombuffer(message.data, dtype=dtype, count=count)
    points = np.column_stack((values["x"], values["y"], values["z"]))
    return np.asarray(points[np.isfinite(points).all(axis=1)], dtype=np.float64)


def voxel_keys(points: np.ndarray, resolution: float) -> np.ndarray:
    if len(points) == 0:
        return np.empty((0, 3), dtype=np.int32)
    return np.floor(np.asarray(points) / resolution).astype(np.int32)


def unique_rows(values: np.ndarray) -> np.ndarray:
    if len(values) == 0:
        return values
    return np.unique(values, axis=0)


@dataclass
class CandidateEvidence:
    blocked_voxels: set[tuple[int, int, int]] = field(default_factory=set)
    margin_voxels: set[tuple[int, int, int]] = field(default_factory=set)
    nearest_obstacle_m: float = np.inf


class DynamicOccupancyMap:
    """Endpoint occupancy with baseline differencing and candidate evidence."""

    def __init__(self, baseline_points: np.ndarray, zones: list[dict], resolution: float = 0.75,
                 gps_margin_m: float = 2.0, obstacle_min_height_m: float = 0.35,
                 vertical_clearance_m: float = 20.0, min_obstacle_voxels: int = 3) -> None:
        self.resolution = float(resolution)
        self.gps_margin_m = float(gps_margin_m)
        self.obstacle_min_height_m = float(obstacle_min_height_m)
        self.vertical_clearance_m = float(vertical_clearance_m)
        self.min_obstacle_voxels = int(min_obstacle_voxels)
        baseline = unique_rows(voxel_keys(baseline_points, self.resolution))
        self.baseline_keys = {tuple(int(v) for v in row) for row in baseline}
        self.live_hits: dict[tuple[int, int, int], int] = {}
        self.zones = zones
        self.evidence = {int(zone["rank"]): CandidateEvidence() for zone in zones}
        self._cell_size = max(5.0, max(float(zone["radius"]) for zone in zones) + self.gps_margin_m)
        self._zone_grid: dict[tuple[int, int], list[dict]] = defaultdict(list)
        for zone in zones:
            reach = float(zone["radius"]) + self.gps_margin_m
            x0 = int(np.floor((float(zone["x"]) - reach) / self._cell_size))
            x1 = int(np.floor((float(zone["x"]) + reach) / self._cell_size))
            y0 = int(np.floor((float(zone["y"]) - reach) / self._cell_size))
            y1 = int(np.floor((float(zone["y"]) + reach) / self._cell_size))
            for gx in range(x0, x1 + 1):
                for gy in range(y0, y1 + 1):
                    self._zone_grid[(gx, gy)].append(zone)

    def update(self, points: np.ndarray) -> int:
        keys = unique_rows(voxel_keys(points, self.resolution))
        changed = 0
        for row in keys:
            key = tuple(int(v) for v in row)
            hits = self.live_hits.get(key, 0) + 1
            self.live_hits[key] = hits
            if hits != self.min_obstacle_voxels or key in self.baseline_keys:
                continue
            changed += 1
            center = (np.asarray(key, dtype=np.float64) + 0.5) * self.resolution
            grid_key = (int(np.floor(center[0] / self._cell_size)),
                        int(np.floor(center[1] / self._cell_size)))
            for zone in self._zone_grid.get(grid_key, []):
                dz = center[2] - float(zone["z"])
                if dz < self.obstacle_min_height_m or dz > self.vertical_clearance_m:
                    continue
                distance = float(np.hypot(center[0] - float(zone["x"]),
                                          center[1] - float(zone["y"])))
                radius = float(zone["radius"])
                evidence = self.evidence[int(zone["rank"])]
                if distance <= radius:
                    evidence.blocked_voxels.add(key)
                    evidence.nearest_obstacle_m = min(evidence.nearest_obstacle_m, distance)
                elif distance <= radius + self.gps_margin_m:
                    evidence.margin_voxels.add(key)
                    evidence.nearest_obstacle_m = min(evidence.nearest_obstacle_m, distance)
        return changed

    def classified_zones(self) -> tuple[list[dict], list[dict]]:
        active: list[dict] = []
        blocked: list[dict] = []
        for source in self.zones:
            zone = dict(source)
            evidence = self.evidence[int(zone["rank"])]
            zone["dynamic_obstacle_voxels"] = len(evidence.blocked_voxels)
            zone["dynamic_margin_voxels"] = len(evidence.margin_voxels)
            zone["dynamic_nearest_obstacle_m"] = evidence.nearest_obstacle_m
            if evidence.blocked_voxels:
                zone["dynamic_state"] = "blocked"
                blocked.append(zone)
            else:
                if evidence.margin_voxels:
                    zone["category"] = "risky"
                    zone["dynamic_state"] = "margin"
                else:
                    zone["dynamic_state"] = "unchanged"
                active.append(zone)
        return active, blocked

    def occupied_centers(self, include_baseline: bool = True, confirmed_only: bool = True) -> np.ndarray:
        keys: set[tuple[int, int, int]] = set(self.baseline_keys) if include_baseline else set()
        threshold = self.min_obstacle_voxels if confirmed_only else 1
        keys.update(key for key, hits in self.live_hits.items() if hits >= threshold)
        if not include_baseline:
            keys.difference_update(self.baseline_keys)
        if not keys:
            return np.empty((0, 3), dtype=np.float64)
        values = np.asarray(list(keys), dtype=np.float64)
        return (values + 0.5) * self.resolution
