#!/usr/bin/env python3
"""Fuse repeated point observations into probabilistic voxel surfels."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from compare_maps import binary_pcd_xyz


def grouped_moments(points: np.ndarray, voxel_size: float):
    cells = np.floor(points / voxel_size).astype(np.int64)
    minimum = cells.min(axis=0)
    shifted = cells - minimum
    widths = shifted.max(axis=0) + 1
    bits = np.maximum(1, np.ceil(np.log2(widths)).astype(int))
    if int(np.sum(bits)) > 63:
        raise ValueError("Voxel index extent exceeds 63-bit packed-key capacity")
    keys = (shifted[:, 0] << int(bits[1] + bits[2])) | (shifted[:, 1] << int(bits[2])) | shifted[:, 2]
    order = np.argsort(keys)
    sorted_keys = keys[order]
    starts = np.r_[0, np.flatnonzero(sorted_keys[1:] != sorted_keys[:-1]) + 1]
    counts = np.diff(np.r_[starts, len(order)]).astype(np.uint32)
    ordered = points[order]
    sums = np.add.reduceat(ordered, starts, axis=0)
    means = sums / counts[:, None]
    products = np.column_stack((
        ordered[:, 0] * ordered[:, 0], ordered[:, 0] * ordered[:, 1],
        ordered[:, 0] * ordered[:, 2], ordered[:, 1] * ordered[:, 1],
        ordered[:, 1] * ordered[:, 2], ordered[:, 2] * ordered[:, 2],
    ))
    second = np.add.reduceat(products, starts, axis=0) / counts[:, None]
    covariance = np.empty((len(starts), 3, 3), dtype=np.float64)
    covariance[:, 0, 0] = second[:, 0] - means[:, 0] ** 2
    covariance[:, 0, 1] = covariance[:, 1, 0] = second[:, 1] - means[:, 0] * means[:, 1]
    covariance[:, 0, 2] = covariance[:, 2, 0] = second[:, 2] - means[:, 0] * means[:, 2]
    covariance[:, 1, 1] = second[:, 3] - means[:, 1] ** 2
    covariance[:, 1, 2] = covariance[:, 2, 1] = second[:, 4] - means[:, 1] * means[:, 2]
    covariance[:, 2, 2] = second[:, 5] - means[:, 2] ** 2
    return means, covariance, counts


def write_surfel_pcd(path: Path, means: np.ndarray, covariance: np.ndarray,
                     normals: np.ndarray, normal_std: np.ndarray, counts: np.ndarray) -> None:
    names = [
        "x", "y", "z", "normal_x", "normal_y", "normal_z", "normal_std",
        "cov_xx", "cov_xy", "cov_xz", "cov_yy", "cov_yz", "cov_zz", "count",
    ]
    dtype = np.dtype([(name, "<u4" if name == "count" else "<f4") for name in names])
    output = np.empty(len(means), dtype=dtype)
    for index, name in enumerate(("x", "y", "z")):
        output[name] = means[:, index]
    for index, name in enumerate(("normal_x", "normal_y", "normal_z")):
        output[name] = normals[:, index]
    output["normal_std"] = normal_std
    for name, row, column in (
        ("cov_xx", 0, 0), ("cov_xy", 0, 1), ("cov_xz", 0, 2),
        ("cov_yy", 1, 1), ("cov_yz", 1, 2), ("cov_zz", 2, 2),
    ):
        output[name] = covariance[:, row, column]
    output["count"] = counts
    sizes = " ".join("4" for _ in names)
    types = " ".join("U" if name == "count" else "F" for name in names)
    header = (
        "# Probabilistic surfel map\nVERSION 0.7\nFIELDS " + " ".join(names) + "\n"
        f"SIZE {sizes}\nTYPE {types}\nCOUNT " + " ".join("1" for _ in names) + "\n"
        f"WIDTH {len(output)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(output)}\nDATA binary\n"
    )
    with path.open("wb") as handle:
        handle.write(header.encode("ascii"))
        handle.write(output.tobytes())


def main() -> int:
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path,
                        default=root / "pcds/bag_only_kiss_icp_reconstructed.pcd")
    parser.add_argument("--output", type=Path,
                        default=root / "pcds/bag_only_kiss_icp_surfels_10cm.pcd")
    parser.add_argument("--diagnostics", type=Path,
                        default=root / "pcds/bag_only_kiss_icp_surfels_10cm.json")
    parser.add_argument("--voxel-size", type=float, default=0.10)
    parser.add_argument("--minimum-normal-observations", type=int, default=5)
    args = parser.parse_args()

    points = np.asarray(binary_pcd_xyz(args.input), dtype=np.float64)
    points = points[np.isfinite(points).all(axis=1)]
    means, covariance, counts = grouped_moments(points, args.voxel_size)
    normals = np.zeros_like(means)
    normal_std = np.full(len(means), np.nan)
    valid = counts >= args.minimum_normal_observations
    indices = np.flatnonzero(valid)
    for begin in range(0, len(indices), 100000):
        selected = indices[begin:begin + 100000]
        eigenvalues, eigenvectors = np.linalg.eigh(covariance[selected])
        block_normals = eigenvectors[:, :, 0]
        block_normals[block_normals[:, 2] < 0.0] *= -1.0
        normals[selected] = block_normals
        normal_std[selected] = np.sqrt(np.maximum(eigenvalues[:, 0], 0.0))
    write_surfel_pcd(args.output, means, covariance, normals, normal_std, counts)
    result = {
        "method": "probabilistic voxel surfel fusion",
        "uses_bayern_reference": False,
        "input_points": len(points),
        "surfel_count": len(means),
        "voxel_size_m": args.voxel_size,
        "surfels_with_normals": int(np.count_nonzero(valid)),
        "observation_count": {
            "median": float(np.median(counts)),
            "p95": float(np.percentile(counts, 95)),
            "maximum": int(np.max(counts)),
        },
        "normal_std_m": {
            "median": float(np.nanmedian(normal_std)),
            "p95": float(np.nanpercentile(normal_std, 95)),
        },
        "note": "The raw reconstructed map remains the master artifact; surfels are a statistical fusion product.",
    }
    args.diagnostics.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
