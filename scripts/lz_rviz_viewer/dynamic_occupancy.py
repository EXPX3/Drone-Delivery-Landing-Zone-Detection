#!/usr/bin/env python3
"""Sparse live occupancy and conservative landing-zone revalidation."""

from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass, field

import numpy as np
from scipy.spatial import cKDTree


def apply_centered_transform(points: np.ndarray, matrix: np.ndarray, origin: np.ndarray) -> np.ndarray:
    """Apply a homogeneous transform around a full 3D registration pivot."""
    points = np.asarray(points, dtype=np.float64)
    origin = np.asarray(origin, dtype=np.float64)
    centered = points - origin
    homogeneous = np.column_stack((centered, np.ones(len(centered))))
    return (np.asarray(matrix) @ homogeneous.T).T[:, :3] + origin


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


class TerrainHeightModel:
    """Robust local ground elevation from a classified ground point cloud."""

    def __init__(self, ground_points: np.ndarray, neighbors: int = 12) -> None:
        points = np.asarray(ground_points, dtype=np.float64)
        if len(points) < neighbors:
            raise ValueError("Ground model has too few points")
        self.points = points
        self.neighbors = int(neighbors)
        self.tree = cKDTree(points[:, :2])

    def height(self, x: float, y: float, maximum_distance_m: float = 4.0) -> float | None:
        distances, indices = self.tree.query([x, y], k=self.neighbors)
        distances = np.atleast_1d(distances)
        indices = np.atleast_1d(indices)
        keep = np.isfinite(distances) & (distances <= maximum_distance_m)
        if np.count_nonzero(keep) < 3:
            return None
        return float(np.median(self.points[indices[keep], 2]))


@dataclass
class CandidateEvidence:
    blocked_voxels: set[tuple[int, int, int]] = field(default_factory=set)
    margin_voxels: set[tuple[int, int, int]] = field(default_factory=set)
    nearest_obstacle_m: float = np.inf


class DynamicOccupancyMap:
    """Endpoint occupancy with baseline differencing and candidate evidence."""

    def __init__(self, baseline_points: np.ndarray, zones: list[dict], resolution: float = 0.75,
                 gps_margin_m: float = 2.0, obstacle_min_height_m: float = 0.35,
                 vertical_clearance_m: float = 20.0, min_obstacle_voxels: int = 3,
                 coarse_resolution: float | None = None,
                 interest_padding_m: float = 1.0, confirmation_window_scans: int = 3,
                 clustering: dict | None = None, hazard_clear_after_scans: int = 15,
                 hazard_clearing_radius_m: float = 20.0) -> None:
        self.resolution = float(resolution)
        self.coarse_resolution = float(coarse_resolution or resolution)
        if self.coarse_resolution < self.resolution:
            raise ValueError("Coarse occupancy resolution cannot be finer than the interest resolution")
        self.gps_margin_m = float(gps_margin_m)
        self.obstacle_min_height_m = float(obstacle_min_height_m)
        self.vertical_clearance_m = float(vertical_clearance_m)
        self.min_obstacle_voxels = int(min_obstacle_voxels)
        self.confirmation_window_scans = int(confirmation_window_scans)
        self.hazard_clear_after_scans = int(hazard_clear_after_scans)
        self.hazard_clearing_radius_m = float(hazard_clearing_radius_m)
        self.clustering = clustering or {"enabled": False}
        self.interest_padding_m = float(interest_padding_m)
        self._zone_centers = np.asarray(
            [[float(zone["x"]), float(zone["y"])] for zone in zones], dtype=np.float64,
        )
        self._zone_reach = np.asarray(
            [float(zone["radius"]) + self.gps_margin_m + self.interest_padding_m for zone in zones],
            dtype=np.float64,
        )
        self._zone_tree = cKDTree(self._zone_centers)
        baseline_interest = self._interest_mask(np.asarray(baseline_points))
        fine_baseline = unique_rows(voxel_keys(baseline_points[baseline_interest], self.resolution))
        coarse_baseline = unique_rows(voxel_keys(
            baseline_points[~baseline_interest], self.coarse_resolution,
        ))
        self.baseline_keys = {tuple(int(v) for v in row) for row in fine_baseline}
        self.coarse_baseline_keys = {tuple(int(v) for v in row) for row in coarse_baseline}
        self._baseline_fine_centers = (fine_baseline.astype(np.float64) + 0.5) * self.resolution
        self._baseline_coarse_centers = (
            coarse_baseline.astype(np.float64) + 0.5
        ) * self.coarse_resolution
        self.live_hits: dict[tuple[int, int, int], int] = {}
        self.coarse_live_hits: dict[tuple[int, int, int], int] = {}
        self.hazard_observations: dict[tuple[int, int, int], list[int]] = defaultdict(list)
        self.confirmed_hazards: set[tuple[int, int, int]] = set()
        self.hazard_last_seen: dict[tuple[int, int, int], int] = {}
        self._hazard_zone_links: dict[
            tuple[int, int, int], list[tuple[int, str]]
        ] = defaultdict(list)
        self.scan_index = 0
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

    def _interest_mask(self, points: np.ndarray) -> np.ndarray:
        if len(points) == 0:
            return np.zeros(0, dtype=bool)
        distance, index = self._zone_tree.query(points[:, :2], k=1)
        return distance <= self._zone_reach[index]

    @property
    def baseline_voxel_count(self) -> int:
        return len(self.baseline_keys) + len(self.coarse_baseline_keys)

    def update(self, points: np.ndarray, sensor_origin: np.ndarray | None = None,
               hazard_points: np.ndarray | None = None) -> int:
        points = np.asarray(points, dtype=np.float64)
        hazard_source = points if hazard_points is None else np.asarray(hazard_points, dtype=np.float64)
        self.scan_index += 1
        interest = self._interest_mask(points)
        keys = unique_rows(voxel_keys(points[interest], self.resolution))
        hazard_interest = self._interest_mask(hazard_source)
        hazard_points = self._clustered_hazard_points(hazard_source[hazard_interest], sensor_origin)
        hazard_positions: dict[tuple[int, int, int], list[np.ndarray]] = defaultdict(list)
        for point, row in zip(hazard_points, voxel_keys(hazard_points, self.resolution)):
            hazard_positions[tuple(int(value) for value in row)].append(point)
        hazard_keys = set(hazard_positions)
        coarse_keys = unique_rows(voxel_keys(points[~interest], self.coarse_resolution))
        for row in coarse_keys:
            key = tuple(int(v) for v in row)
            self.coarse_live_hits[key] = self.coarse_live_hits.get(key, 0) + 1
        changed = 0
        for row in keys:
            key = tuple(int(v) for v in row)
            hits = self.live_hits.get(key, 0) + 1
            self.live_hits[key] = hits
        # Safety evidence is deliberately independent of map/display sampling.
        # Otherwise a valid obstacle can disappear when the fused map is thinned.
        for key in hazard_keys:
            nearby_confirmed = [
                (key[0] + dx, key[1] + dy, key[2] + dz)
                for dx in (-1, 0, 1) for dy in (-1, 0, 1) for dz in (-1, 0, 1)
                if (key[0] + dx, key[1] + dy, key[2] + dz) in self.confirmed_hazards
            ]
            if nearby_confirmed:
                for neighbor in nearby_confirmed:
                    self.hazard_last_seen[neighbor] = self.scan_index
                continue
            cutoff = self.scan_index - self.confirmation_window_scans + 1
            nearby_scans: set[int] = {self.scan_index}
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for dz in (-1, 0, 1):
                        neighbor = (key[0] + dx, key[1] + dy, key[2] + dz)
                        nearby_scans.update(
                            value for value in self.hazard_observations.get(neighbor, [])
                            if value >= cutoff
                        )
            observations = self.hazard_observations[key]
            if not observations or observations[-1] != self.scan_index:
                observations.append(self.scan_index)
            while observations and observations[0] < cutoff:
                observations.pop(0)
            center = np.mean(hazard_positions[key], axis=0)
            required_hits = self.min_obstacle_voxels
            if sensor_origin is not None:
                distance_to_sensor = float(np.linalg.norm(center[:2] - np.asarray(sensor_origin)[:2]))
                if distance_to_sensor > float(self.clustering.get("far_range_m", np.inf)):
                    required_hits = int(self.clustering.get("beyond_far_confirmation_hits", required_hits))
            if len(nearby_scans) < required_hits:
                continue
            self.confirmed_hazards.add(key)
            self.hazard_last_seen[key] = self.scan_index
            changed += 1
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
                    self._hazard_zone_links[key].append((int(zone["rank"]), "blocked"))
                    evidence.nearest_obstacle_m = min(evidence.nearest_obstacle_m, distance)
                elif distance <= radius + self.gps_margin_m:
                    evidence.margin_voxels.add(key)
                    self._hazard_zone_links[key].append((int(zone["rank"]), "margin"))
                    evidence.nearest_obstacle_m = min(evidence.nearest_obstacle_m, distance)
        changed += self._clear_stale_hazards(sensor_origin)
        return changed

    def _clear_stale_hazards(self, sensor_origin: np.ndarray | None) -> int:
        """Restore candidates only after a nearby area has remained clear."""
        if sensor_origin is None or self.hazard_clear_after_scans <= 0:
            return 0
        origin = np.asarray(sensor_origin, dtype=np.float64)
        stale = [
            key for key in self.confirmed_hazards
            if self.scan_index - self.hazard_last_seen.get(key, self.scan_index)
            >= self.hazard_clear_after_scans
            and np.linalg.norm(
                (np.asarray(key[:2], dtype=np.float64) + 0.5) * self.resolution
                - origin[:2]
            ) <= self.hazard_clearing_radius_m
        ]
        for key in stale:
            self.confirmed_hazards.remove(key)
            self.hazard_last_seen.pop(key, None)
            for rank, kind in self._hazard_zone_links.pop(key, []):
                evidence = self.evidence[rank]
                target = evidence.blocked_voxels if kind == "blocked" else evidence.margin_voxels
                target.discard(key)
                if not evidence.blocked_voxels and not evidence.margin_voxels:
                    evidence.nearest_obstacle_m = np.inf
        return len(stale)

    def _clustered_hazard_points(self, points: np.ndarray,
                                 sensor_origin: np.ndarray | None) -> np.ndarray:
        if not len(points):
            return points
        distance, zone_index = self._zone_tree.query(points[:, :2], k=1)
        zone_z = np.asarray([float(self.zones[index]["z"]) for index in zone_index])
        reach = np.asarray([
            float(self.zones[index]["radius"]) + self.gps_margin_m for index in zone_index
        ])
        dz = points[:, 2] - zone_z
        keep = (
            (distance <= reach) & (dz >= self.obstacle_min_height_m)
            & (dz <= self.vertical_clearance_m)
        )
        hazards = points[keep]
        if not self.clustering.get("enabled", False) or len(hazards) == 0:
            return hazards

        radius = float(self.clustering["radius_m"])
        tree = cKDTree(hazards)
        pairs = tree.query_pairs(radius, output_type="ndarray")
        parent = np.arange(len(hazards))

        def find(value: int) -> int:
            while parent[value] != value:
                parent[value] = parent[parent[value]]
                value = int(parent[value])
            return value

        for first, second in pairs:
            root_first, root_second = find(int(first)), find(int(second))
            if root_first != root_second:
                parent[root_second] = root_first
        labels = np.asarray([find(index) for index in range(len(hazards))])
        unique, counts = np.unique(labels, return_counts=True)
        accepted: list[np.ndarray] = []
        for label, count in zip(unique, counts):
            cluster = hazards[labels == label]
            if sensor_origin is None:
                horizontal_range = 0.0
            else:
                horizontal_range = float(np.linalg.norm(
                    np.median(cluster[:, :2], axis=0) - np.asarray(sensor_origin)[:2]
                ))
            if horizontal_range <= float(self.clustering["near_range_m"]):
                minimum = int(self.clustering["minimum_points_near"])
            elif horizontal_range <= float(self.clustering["mid_range_m"]):
                minimum = int(self.clustering["minimum_points_mid"])
            else:
                minimum = int(self.clustering["minimum_points_far"])
            if count >= minimum:
                accepted.append(cluster)
        return np.vstack(accepted) if accepted else np.empty((0, 3), dtype=np.float64)

    def occupancy_layers(self, include_baseline: bool = True,
                         confirmed_only: bool = True,
                         center: np.ndarray | None = None,
                         horizontal_radius_m: float | None = None) -> list[tuple[str, np.ndarray, float]]:
        threshold = self.min_obstacle_voxels if confirmed_only else 1
        fine: set[tuple[int, int, int]] = set()
        fine.update(key for key, hits in self.live_hits.items() if hits >= threshold)
        coarse: set[tuple[int, int, int]] = set()
        coarse.update(key for key, hits in self.coarse_live_hits.items() if hits >= threshold)
        if not include_baseline:
            fine.difference_update(self.baseline_keys)
            coarse.difference_update(self.coarse_baseline_keys)

        def centers(keys: set[tuple[int, int, int]], resolution: float) -> np.ndarray:
            if not keys:
                return np.empty((0, 3), dtype=np.float64)
            return (np.asarray(list(keys), dtype=np.float64) + 0.5) * resolution

        fine_centers = centers(fine, self.resolution)
        coarse_centers = centers(coarse, self.coarse_resolution)
        if include_baseline:
            fine_centers = np.vstack((self._baseline_fine_centers, fine_centers))
            coarse_centers = np.vstack((self._baseline_coarse_centers, coarse_centers))
        if center is not None and horizontal_radius_m is not None:
            center = np.asarray(center, dtype=np.float64)
            radius_squared = float(horizontal_radius_m) ** 2
            fine_centers = fine_centers[
                np.sum((fine_centers[:, :2] - center[:2]) ** 2, axis=1) <= radius_squared
            ]
            coarse_centers = coarse_centers[
                np.sum((coarse_centers[:, :2] - center[:2]) ** 2, axis=1) <= radius_squared
            ]
        return [("fine", fine_centers, self.resolution),
                ("coarse", coarse_centers, self.coarse_resolution)]

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

    def add_confirmed_lidar_obstacles(self, points: np.ndarray) -> int:
        """Apply independently confirmed above-ground LiDAR points as safety evidence."""
        changed = 0
        for point in np.asarray(points, dtype=np.float64):
            key = tuple(int(value) for value in np.floor(point / self.resolution))
            grid_key = (int(np.floor(point[0] / self._cell_size)),
                        int(np.floor(point[1] / self._cell_size)))
            for zone in self._zone_grid.get(grid_key, []):
                dz = point[2] - float(zone["z"])
                if dz < self.obstacle_min_height_m or dz > self.vertical_clearance_m:
                    continue
                distance = float(np.hypot(point[0] - float(zone["x"]),
                                          point[1] - float(zone["y"])))
                evidence = self.evidence[int(zone["rank"])]
                target = None
                if distance <= float(zone["radius"]):
                    target = evidence.blocked_voxels
                elif distance <= float(zone["radius"]) + self.gps_margin_m:
                    target = evidence.margin_voxels
                if target is not None and key not in target:
                    target.add(key)
                    evidence.nearest_obstacle_m = min(evidence.nearest_obstacle_m, distance)
                    changed += 1
        return changed

    def occupied_centers(self, include_baseline: bool = True, confirmed_only: bool = True) -> np.ndarray:
        layers = self.occupancy_layers(include_baseline, confirmed_only)
        populated = [centers for _, centers, _ in layers if len(centers)]
        return np.vstack(populated) if populated else np.empty((0, 3), dtype=np.float64)
