#!/usr/bin/env python3
"""Ground-reference models for physically validating semantic detections."""

from __future__ import annotations

from dataclasses import dataclass
import time

import numpy as np


BAYERN_GROUND_REFERENCE = "bayern_ground_reference"
LOCAL_DETECTION_GROUND_REFERENCE = "local_detection_ground_reference"
GROUND_REFERENCE_MODES = {
    BAYERN_GROUND_REFERENCE,
    LOCAL_DETECTION_GROUND_REFERENCE,
}


@dataclass(frozen=True)
class GroundEstimate:
    ground_z: np.ndarray
    diagnostics: dict


def bayern_ground_estimate(points: np.ndarray, terrain: object) -> GroundEstimate:
    """Evaluate the existing classified Bayern terrain model."""
    started = time.perf_counter()
    ground = np.asarray([
        terrain.height(float(point[0]), float(point[1])) for point in points
    ], dtype=object)
    values = np.asarray([
        np.nan if value is None else float(value) for value in ground
    ], dtype=np.float64)
    return GroundEstimate(values, {
        "mode": BAYERN_GROUND_REFERENCE,
        "fit_success": bool(np.isfinite(values).any()),
        "query_points": int(len(points)),
        "fit_time_ms": float((time.perf_counter() - started) * 1000.0),
    })


def _plane_z(points_xy: np.ndarray, coefficients: np.ndarray,
             center_xy: np.ndarray) -> np.ndarray:
    relative = np.asarray(points_xy, dtype=np.float64) - center_xy
    return relative @ coefficients[:2] + coefficients[2]


def local_detection_ground_estimate(
        detection_points: np.ndarray, scan_points: np.ndarray, radius_m: float = 5.0,
        distance_threshold_m: float = 0.15, maximum_slope_deg: float = 20.0,
        minimum_inliers: int = 20, iterations: int = 96) -> GroundEstimate:
    """Fit a robust local ground plane independently around one detection.

    RANSAC finds the dominant near-horizontal plane in the current registered
    LiDAR scan. The final least-squares fit uses only its inliers.
    """
    started = time.perf_counter()
    detection_points = np.asarray(detection_points, dtype=np.float64)
    scan_points = np.asarray(scan_points, dtype=np.float64)
    empty = np.full(len(detection_points), np.nan, dtype=np.float64)
    base = {
        "mode": LOCAL_DETECTION_GROUND_REFERENCE,
        "radius_m": float(radius_m),
        "fit_success": False,
        "query_points": int(len(detection_points)),
    }
    if not len(detection_points):
        base["failure"] = "no_projected_detection_points"
        base["fit_time_ms"] = float((time.perf_counter() - started) * 1000.0)
        return GroundEstimate(empty, base)
    if len(scan_points) < minimum_inliers:
        base["failure"] = "insufficient_scan_points"
        base["fit_time_ms"] = float((time.perf_counter() - started) * 1000.0)
        return GroundEstimate(empty, base)

    center_xy = np.median(detection_points[:, :2], axis=0)
    local = scan_points[
        np.linalg.norm(scan_points[:, :2] - center_xy, axis=1) <= radius_m
    ]
    local = local[np.isfinite(local).all(axis=1)]
    base["local_points"] = int(len(local))
    if len(local) < minimum_inliers:
        base["failure"] = "insufficient_local_points"
        base["fit_time_ms"] = float((time.perf_counter() - started) * 1000.0)
        return GroundEstimate(empty, base)

    xy = local[:, :2] - center_xy
    design = np.column_stack((xy, np.ones(len(local))))
    maximum_gradient = float(np.tan(np.radians(maximum_slope_deg)))
    rng_seed = int(np.round(np.sum(center_xy) * 1000.0)) & 0xFFFFFFFF
    rng = np.random.default_rng(rng_seed)
    best = np.zeros(len(local), dtype=bool)
    best_error = np.inf
    for _ in range(iterations):
        sample = rng.choice(len(local), 3, replace=False)
        try:
            coefficients = np.linalg.solve(design[sample], local[sample, 2])
        except np.linalg.LinAlgError:
            continue
        if np.linalg.norm(coefficients[:2]) > maximum_gradient:
            continue
        residual = np.abs(local[:, 2] - design @ coefficients)
        inliers = residual <= distance_threshold_m
        count = int(np.count_nonzero(inliers))
        if count < minimum_inliers:
            continue
        error = float(np.median(residual[inliers]))
        if count > np.count_nonzero(best) or (count == np.count_nonzero(best) and error < best_error):
            best, best_error = inliers, error

    if np.count_nonzero(best) < minimum_inliers:
        base["failure"] = "no_dominant_near_horizontal_plane"
        base["fit_time_ms"] = float((time.perf_counter() - started) * 1000.0)
        return GroundEstimate(empty, base)

    coefficients, *_ = np.linalg.lstsq(design[best], local[best, 2], rcond=None)
    if np.linalg.norm(coefficients[:2]) > maximum_gradient:
        base["failure"] = "refined_plane_too_steep"
        base["fit_time_ms"] = float((time.perf_counter() - started) * 1000.0)
        return GroundEstimate(empty, base)
    residual = local[best, 2] - design[best] @ coefficients
    base.update({
        "fit_success": True,
        "inliers": int(np.count_nonzero(best)),
        "inlier_fraction": float(np.mean(best)),
        "residual_median_abs_m": float(np.median(np.abs(residual))),
        "residual_p95_abs_m": float(np.percentile(np.abs(residual), 95)),
        "slope_deg": float(np.degrees(np.arctan(np.linalg.norm(coefficients[:2])))),
        "coefficients_centered_z_ax_by_c": coefficients.tolist(),
        "center_xy": center_xy.tolist(),
        "fit_time_ms": float((time.perf_counter() - started) * 1000.0),
    })
    return GroundEstimate(
        _plane_z(detection_points[:, :2], coefficients, center_xy), base,
    )


def estimate_detection_ground(mode: str, detection_points: np.ndarray,
                              scan_points: np.ndarray, terrain: object,
                              config: dict) -> GroundEstimate:
    if mode == BAYERN_GROUND_REFERENCE:
        return bayern_ground_estimate(detection_points, terrain)
    if mode == LOCAL_DETECTION_GROUND_REFERENCE:
        return local_detection_ground_estimate(
            detection_points, scan_points,
            radius_m=float(config.get("radius_m", 5.0)),
            distance_threshold_m=float(config.get("ransac_distance_threshold_m", 0.15)),
            maximum_slope_deg=float(config.get("maximum_slope_deg", 20.0)),
            minimum_inliers=int(config.get("minimum_inliers", 20)),
            iterations=int(config.get("ransac_iterations", 96)),
        )
    raise ValueError(f"Unknown semantic ground-reference mode: {mode}")
