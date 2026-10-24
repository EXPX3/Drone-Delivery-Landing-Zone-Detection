#!/usr/bin/env python3
"""Bundle-adjust scan Z offsets from repeated low-surface observations."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import yaml
from rosbags.highlevel import AnyReader
from scipy.sparse import coo_matrix, vstack
from scipy.sparse.linalg import lsqr

SCRIPTS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPTS / "bag_lio_mapping"))

from reconstruct_from_poses import transform_timed_points  # noqa: E402
from run_kiss_icp_mapping import quaternion_matrix, valid_body_scan  # noqa: E402


def scan_cell_minima(points: np.ndarray, cell_size: float) -> tuple[np.ndarray, np.ndarray]:
    cells = np.floor(points[:, :2] / cell_size).astype(np.int64)
    order = np.lexsort((points[:, 2], cells[:, 1], cells[:, 0]))
    sorted_cells = cells[order]
    starts = np.r_[0, np.flatnonzero(
        (sorted_cells[1:, 0] != sorted_cells[:-1, 0])
        | (sorted_cells[1:, 1] != sorted_cells[:-1, 1])
    ) + 1]
    return sorted_cells[starts], points[order[starts], 2]


def solve_offsets(scan_index: np.ndarray, cell_index: np.ndarray, heights: np.ndarray,
                  scan_count: int, cell_count: int, smooth_weight: float,
                  anchor_scans: int, robust_iterations: int) -> tuple[np.ndarray, np.ndarray]:
    observation_count = len(heights)
    rows = np.repeat(np.arange(observation_count), 2)
    columns = np.column_stack((scan_index, scan_count + cell_index)).ravel()
    values = np.tile([1.0, -1.0], observation_count)
    observation_matrix = coo_matrix(
        (values, (rows, columns)), shape=(observation_count, scan_count + cell_count),
    ).tocsr()
    rhs = -heights

    smooth_rows = scan_count - 2
    sr = np.repeat(np.arange(smooth_rows), 3)
    sc = np.column_stack((np.arange(smooth_rows), np.arange(1, scan_count - 1),
                          np.arange(2, scan_count))).ravel()
    sv = np.tile([smooth_weight, -2.0 * smooth_weight, smooth_weight], smooth_rows)
    smooth = coo_matrix((sv, (sr, sc)), shape=(smooth_rows, scan_count + cell_count)).tocsr()
    anchors = np.unique(np.r_[np.arange(anchor_scans),
                              np.arange(scan_count - anchor_scans, scan_count)])
    anchor = coo_matrix(
        (np.full(len(anchors), 20.0), (np.arange(len(anchors)), anchors)),
        shape=(len(anchors), scan_count + cell_count),
    ).tocsr()
    solution = np.zeros(scan_count + cell_count)
    weights = np.ones(observation_count)
    for _ in range(robust_iterations):
        weighted_observations = observation_matrix.multiply(weights[:, None])
        system = vstack((weighted_observations, smooth, anchor), format="csr")
        target = np.r_[rhs * weights, np.zeros(smooth_rows + len(anchors))]
        solution = lsqr(system, target, atol=1e-7, btol=1e-7, iter_lim=500)[0]
        residual = observation_matrix @ solution - rhs
        scale = max(1.4826 * np.median(np.abs(residual - np.median(residual))), 0.01)
        weights = np.minimum(1.0, 1.5 * scale / np.maximum(np.abs(residual), 1e-12))
    return solution[:scan_count], observation_matrix @ solution - rhs


def main() -> int:
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--poses", type=Path,
                        default=root / "pcds/bag_only_loop_optimized_poses.npz")
    parser.add_argument("--config", type=Path,
                        default=root / "scripts/bag_lio_mapping/kiss_icp_config.yaml")
    parser.add_argument("--output", type=Path,
                        default=root / "pcds/bag_only_strip_optimized_poses.npz")
    parser.add_argument("--diagnostics", type=Path,
                        default=root / "pcds/bag_only_strip_optimization.json")
    parser.add_argument("--cell-size", type=float, default=0.50)
    parser.add_argument("--point-stride", type=int, default=4)
    parser.add_argument("--minimum-cell-scans", type=int, default=4)
    parser.add_argument("--smooth-weight", type=float, default=4.0)
    parser.add_argument("--anchor-scans", type=int, default=100)
    args = parser.parse_args()
    settings = yaml.safe_load(args.config.read_text())
    pose_data = np.load(args.poses)
    poses, pose_times = pose_data["poses"], pose_data["timestamps"]
    extrinsic = settings["extrinsic_base_from_lidar"]
    rotation = quaternion_matrix(extrinsic["quaternion_xyzw"])
    translation = np.asarray(extrinsic["translation_m"], dtype=np.float64)

    all_cells: list[np.ndarray] = []
    all_heights: list[np.ndarray] = []
    all_scans: list[np.ndarray] = []
    previous = poses[0]
    scan = 0
    with AnyReader([args.bag]) as reader:
        connections = [item for item in reader.connections if item.topic == "/velodyne_points"]
        for _, _, raw in reader.messages(connections=connections):
            if scan >= len(poses):
                break
            message = reader.deserialize(raw, connections[0].msgtype)
            points, relative_times = valid_body_scan(message, rotation, translation, 1.0, 80.0)
            points, relative_times = points[::args.point_stride], relative_times[::args.point_stride]
            mapped = transform_timed_points(points, relative_times, previous, poses[scan])
            cells, heights = scan_cell_minima(mapped, args.cell_size)
            all_cells.append(cells)
            all_heights.append(heights)
            all_scans.append(np.full(len(cells), scan, dtype=np.int32))
            previous = poses[scan]
            scan += 1
            if scan % 500 == 0:
                print(f"Extracted low surfaces from {scan} scans", flush=True)
    cells = np.vstack(all_cells)
    heights = np.concatenate(all_heights)
    scan_index = np.concatenate(all_scans)
    unique_cells, cell_index, cell_counts = np.unique(
        cells, axis=0, return_inverse=True, return_counts=True,
    )
    keep = cell_counts[cell_index] >= args.minimum_cell_scans
    heights, scan_index, cell_index = heights[keep], scan_index[keep], cell_index[keep]
    used_cells, remapped = np.unique(cell_index, return_inverse=True)
    offsets, residual = solve_offsets(
        scan_index, remapped, heights, len(poses), len(used_cells),
        args.smooth_weight, args.anchor_scans, robust_iterations=3,
    )
    optimized = poses.copy()
    optimized[:, 2, 3] += offsets
    np.savez_compressed(args.output, poses=optimized, timestamps=pose_times,
                        vertical_offsets=offsets)
    result = {
        "method": "robust repeated-cell vertical strip bundle adjustment",
        "uses_bayern_reference": False,
        "cell_size_m": args.cell_size,
        "observations": len(heights),
        "repeated_cells": len(used_cells),
        "vertical_offset_m": {
            "minimum": float(np.min(offsets)), "maximum": float(np.max(offsets)),
            "median": float(np.median(offsets)),
            "p95_absolute": float(np.percentile(np.abs(offsets), 95)),
        },
        "bundle_residual_m": {
            "rmse": float(np.sqrt(np.mean(residual ** 2))),
            "nmad": float(1.4826 * np.median(np.abs(residual - np.median(residual)))),
            "p95_absolute": float(np.percentile(np.abs(residual), 95)),
        },
        "anchors": f"first and last {args.anchor_scans} grounded scans",
    }
    args.diagnostics.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
