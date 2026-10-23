#!/usr/bin/env python3
"""Compare matched bag-only deskewed and single-pose point maps."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


def binary_pcd_xyz(path: Path) -> np.memmap:
    with path.open("rb") as handle:
        points = None
        while True:
            line = handle.readline()
            if not line:
                raise ValueError(f"Missing DATA line in {path}")
            text = line.decode("ascii").strip()
            if text.startswith("POINTS "):
                points = int(text.split()[1])
            if text == "DATA binary":
                offset = handle.tell()
                break
    if points is None:
        raise ValueError(f"Missing POINTS value in {path}")
    expected = offset + points * 3 * np.dtype("<f4").itemsize
    if path.stat().st_size != expected:
        raise ValueError(f"Expected packed binary XYZ data in {path}")
    return np.memmap(path, dtype="<f4", mode="r", offset=offset, shape=(points, 3))


def distribution(values: np.ndarray) -> dict[str, float]:
    return {
        "median": float(np.median(values)),
        "p90": float(np.percentile(values, 90)),
        "p95": float(np.percentile(values, 95)),
        "maximum": float(np.max(values)),
    }


def column_thickness(points: np.ndarray, cell_size: float,
                     minimum_points: int) -> tuple[np.ndarray, int]:
    """Return robust P90-P10 Z thickness for sufficiently observed XY cells."""
    ix = np.floor(points[:, 0] / cell_size).astype(np.int64)
    iy = np.floor(points[:, 1] / cell_size).astype(np.int64)
    order = np.lexsort((points[:, 2], iy, ix))
    sx, sy = ix[order], iy[order]
    starts = np.r_[0, np.flatnonzero((sx[1:] != sx[:-1]) | (sy[1:] != sy[:-1])) + 1]
    counts = np.diff(np.r_[starts, len(order)])
    valid = counts >= minimum_points
    starts, counts = starts[valid], counts[valid]
    low = starts + np.floor(0.10 * (counts - 1)).astype(np.int64)
    high = starts + np.floor(0.90 * (counts - 1)).astype(np.int64)
    sorted_z = points[order, 2]
    return sorted_z[high] - sorted_z[low], int(len(starts))


def main() -> int:
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--deskewed", type=Path,
                        default=root / "pcds/bag_only_deskewed_map.pcd")
    parser.add_argument("--control", type=Path,
                        default=root / "pcds/bag_only_single_pose_control.pcd")
    parser.add_argument("--output", type=Path,
                        default=root / "pcds/bag_only_deskew_comparison.json")
    parser.add_argument("--sample-stride", type=int, default=4)
    parser.add_argument("--xy-cell", type=float, default=0.25)
    parser.add_argument("--minimum-cell-points", type=int, default=20)
    args = parser.parse_args()

    deskewed = binary_pcd_xyz(args.deskewed)
    control = binary_pcd_xyz(args.control)
    if len(deskewed) != len(control):
        raise ValueError("Matched maps must contain the same number of ordered points")
    sample = slice(None, None, args.sample_stride)
    deskewed_sample = np.asarray(deskewed[sample], dtype=np.float64)
    control_sample = np.asarray(control[sample], dtype=np.float64)
    finite = np.isfinite(deskewed_sample).all(axis=1) & np.isfinite(control_sample).all(axis=1)
    deskewed_sample, control_sample = deskewed_sample[finite], control_sample[finite]

    correction = np.linalg.norm(deskewed_sample - control_sample, axis=1)
    deskewed_thickness, deskewed_cells = column_thickness(
        deskewed_sample, args.xy_cell, args.minimum_cell_points,
    )
    control_thickness, control_cells = column_thickness(
        control_sample, args.xy_cell, args.minimum_cell_points,
    )
    result = {
        "method": "bag-only matched-map internal diagnostic",
        "uses_bayern_reference": False,
        "input_points_each": len(deskewed),
        "sampled_matched_points": len(correction),
        "sample_stride": args.sample_stride,
        "deskew_correction_distance_m": distribution(correction),
        "repeated_xy_column_thickness": {
            "definition": (
                f"P90 minus P10 Z in {args.xy_cell:g} m XY cells; "
                "objects and vegetation also contribute"
            ),
            "xy_cell_m": args.xy_cell,
            "minimum_sampled_points": args.minimum_cell_points,
            "deskewed_cells": deskewed_cells,
            "control_cells": control_cells,
            "deskewed_m": distribution(deskewed_thickness),
            "single_pose_control_m": distribution(control_thickness),
        },
        "interpretation_limit": (
            "Correction distance measures the applied deskew, not accuracy. Column thickness is an "
            "internal repeatability proxy and cannot separate ground, vertical objects, vegetation, "
            "or dynamic objects. Absolute accuracy requires held-out Bayern/control-point evaluation."
        ),
    }
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
