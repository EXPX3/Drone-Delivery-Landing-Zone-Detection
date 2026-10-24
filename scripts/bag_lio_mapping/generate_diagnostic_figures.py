#!/usr/bin/env python3
"""Generate matched before/after figures for the mapping diagnosis report."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

SCRIPTS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPTS / "bag_lio_mapping"))

from compare_maps import binary_pcd_xyz  # noqa: E402


def load_alignment(path: Path) -> np.ndarray:
    return np.genfromtxt(path, delimiter=",", names=True)


def matched_section(path: Path, y_min: float, y_max: float,
                    x_min: float, x_max: float, stride: int) -> np.ndarray:
    points = np.asarray(binary_pcd_xyz(path))[::stride]
    keep = (
        (points[:, 0] >= x_min) & (points[:, 0] <= x_max)
        & (points[:, 1] >= y_min) & (points[:, 1] <= y_max)
        & (points[:, 2] >= -3.0) & (points[:, 2] <= 3.0)
    )
    return points[keep]


def main() -> int:
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--before-map", type=Path,
                        default=root / "pcds/bag_only_loop_optimized_reconstructed.pcd")
    parser.add_argument("--after-map", type=Path,
                        default=root / "pcds/bag_only_strip_optimized_reconstructed.pcd")
    parser.add_argument("--before-csv", type=Path,
                        default=root / "pcds/bag_only_loop_scan_alignment.csv")
    parser.add_argument("--after-csv", type=Path,
                        default=root / "pcds/bag_only_strip_scan_alignment.csv")
    parser.add_argument("--output-dir", type=Path,
                        default=root / "scripts/report_assets/bag_mapping_diagnosis")
    parser.add_argument("--y-min", type=float, default=-2.0)
    parser.add_argument("--y-max", type=float, default=2.0)
    parser.add_argument("--x-min", type=float, default=-15.0)
    parser.add_argument("--x-max", type=float, default=45.0)
    parser.add_argument("--point-stride", type=int, default=5)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    before = load_alignment(args.before_csv)
    after = load_alignment(args.after_csv)
    elapsed = before["time_s"] - before["time_s"][0]
    fig, axis = plt.subplots(figsize=(10.5, 4.0), constrained_layout=True)
    axis.plot(elapsed, before["bias_m"], color="#b33a3a", linewidth=1.1,
              label="Before: loop-only trajectory")
    axis.plot(elapsed, after["bias_m"], color="#147d64", linewidth=1.1,
              label="After: repeated-surface strip bundle")
    axis.axhline(0.0, color="#333333", linewidth=0.7)
    axis.set(xlabel="Elapsed bag time (s)", ylabel="Per-scan vertical ground bias (m)",
             title="Held-out ground error over the complete flight")
    axis.grid(color="#d8d8d8", linewidth=0.6)
    axis.legend(frameon=False, ncol=2, loc="upper center")
    fig.savefig(args.output_dir / "vertical_bias_before_after.png", dpi=220)
    plt.close(fig)

    maps = [
        matched_section(args.before_map, args.y_min, args.y_max,
                        args.x_min, args.x_max, args.point_stride),
        matched_section(args.after_map, args.y_min, args.y_max,
                        args.x_min, args.x_max, args.point_stride),
    ]
    fig, axes = plt.subplots(1, 2, figsize=(12.0, 4.5), sharex=True, sharey=True,
                             constrained_layout=True)
    titles = ["Before: repeated vertical layers", "After: corrected scan alignment"]
    for axis, points, title in zip(axes, maps, titles):
        axis.scatter(points[:, 0], points[:, 2], s=0.18, c=points[:, 2],
                     cmap="viridis", vmin=-2.0, vmax=2.0, alpha=0.42,
                     rasterized=True, linewidths=0)
        axis.set_title(title)
        axis.set_xlabel("Map X (m)")
        axis.set_xlim(args.x_min, args.x_max)
        axis.set_ylim(-2.5, 2.5)
        axis.grid(color="#dddddd", linewidth=0.45)
        axis.set_aspect("auto")
    axes[0].set_ylabel("Map Z (m)")
    fig.suptitle(
        f"Matched point-cloud side section: Y in [{args.y_min:.1f}, {args.y_max:.1f}] m",
        fontsize=12,
    )
    fig.savefig(args.output_dir / "pointcloud_section_before_after.png", dpi=240)
    plt.close(fig)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
