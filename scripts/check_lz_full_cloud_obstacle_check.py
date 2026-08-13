#!/usr/bin/env python3
"""Validate landing-zone candidates against the full point cloud.

This script is intended to be used when the candidate circles are generated
from a ground-only (class 2) point cloud, but vertical obstacle checking
should use the full cropped point cloud containing all LAS classes.

Activation tag in config:
  TAG: USE_GROUND_CLASS2_FOR_DETECTION_AND_FULL_CLOUD_FOR_OBSTACLE_CHECK

Example:
  python3 scripts/check_lz_full_cloud_obstacle_check.py \
    --config lib/config/algo_testing_own_pcd_config.yaml \
    --candidate-csv pcds/region_growing_landing_zones_max2000.csv \
    --full-cloud-pcd pcds/696_5328_cropped_by_bayernatlas_20260812084517.pcd
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import numpy as np
import open3d as o3d

TAG = "USE_GROUND_CLASS2_FOR_DETECTION_AND_FULL_CLOUD_FOR_OBSTACLE_CHECK"
TAG_RADIUS_CAP = "USE_RADIUS_2_5_ONLY_EXPERIMENT"
TAG_EXACT_RADIUS_CAP = "USE_EXACT_RADIUS_2_5_ONLY_EXPERIMENT"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=Path("lib/config/algo_testing_own_pcd_config.yaml"),
        help="Configuration file containing the activation tag.",
    )
    parser.add_argument(
        "--candidate-csv",
        type=Path,
        default=Path("pcds/region_growing_landing_zones_max2000.csv"),
        help="CSV file with landing-zone candidate circles.",
    )
    parser.add_argument(
        "--full-cloud-pcd",
        type=Path,
        default=Path("pcds/696_5328_cropped_by_bayernatlas_20260812084517.pcd"),
        help="Full cropped PCD containing all classes for obstacle checking.",
    )
    parser.add_argument(
        "--threshold",
        type=float,
        default=0.5,
        help="Distance threshold above the plane to count an obstacle point.",
    )
    parser.add_argument(
        "--max-points-above",
        type=int,
        default=20,
        help="Maximum allowed obstacle points above a candidate before rejection.",
    )
    parser.add_argument(
        "--output-csv",
        type=Path,
        default=None,
        help="Output CSV file for filtered landing zones.",
    )
    return parser.parse_args()


def require_activation_tag(config_path: Path) -> str:
    if not config_path.exists():
        raise FileNotFoundError(f"Config file not found: {config_path}")
    text = config_path.read_text(encoding="utf-8")
    if TAG not in text and TAG_RADIUS_CAP not in text and TAG_EXACT_RADIUS_CAP not in text:
        raise RuntimeError(
            f"Activation tag not found in {config_path}.\n"
            f"Add one of the following lines to activate the workflow:\n"
            f"# TAG: {TAG}\n"
            f"# TAG: {TAG_RADIUS_CAP}\n"
            f"# TAG: {TAG_EXACT_RADIUS_CAP}"
        )
    return text


def load_candidates(csv_path: Path) -> list[dict[str, float]]:
    candidates = []
    with csv_path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            candidates.append(
                {
                    "rank": int(float(row["rank"])),
                    "x": float(row["x"]),
                    "y": float(row["y"]),
                    "z": float(row["z"]),
                    "radius": float(row["radius"]),
                    "normal_x": float(row["normal_x"]),
                    "normal_y": float(row["normal_y"]),
                    "normal_z": float(row["normal_z"]),
                }
            )
    return candidates


def load_point_cloud(pcd_path: Path) -> np.ndarray:
    point_cloud = o3d.io.read_point_cloud(str(pcd_path))
    if point_cloud.is_empty():
        raise RuntimeError(f"Full cloud PCD is empty or unreadable: {pcd_path}")
    return np.asarray(point_cloud.points, dtype=np.float64)


def check_candidate(
    candidate: dict[str, float],
    full_points: np.ndarray,
    threshold: float,
    max_points_above: int,
) -> tuple[bool, int]:
    center = np.array([candidate["x"], candidate["y"], candidate["z"]], dtype=np.float64)
    normal = np.array([candidate["normal_x"], candidate["normal_y"], candidate["normal_z"]], dtype=np.float64)
    normal_norm = np.linalg.norm(normal)
    if normal_norm < 1e-9:
        raise RuntimeError(f"Invalid normal vector for candidate at rank {candidate['rank']}")
    normal /= normal_norm

    diffs = full_points - center
    horizontal_dist = np.linalg.norm(diffs[:, :2], axis=1)
    in_radius = horizontal_dist <= candidate["radius"]
    if not np.any(in_radius):
        return True, 0

    heights = diffs @ normal
    points_above = np.count_nonzero(np.logical_and(in_radius, heights > threshold))
    return points_above < max_points_above, points_above


def write_filtered_csv(
    candidates: list[dict[str, float]],
    output_path: Path,
) -> None:
    fieldnames = ["rank", "x", "y", "z", "radius", "normal_x", "normal_y", "normal_z"]
    with output_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for candidate in candidates:
            writer.writerow({k: candidate[k] for k in fieldnames})


def main() -> int:
    args = parse_args()
    config_text = require_activation_tag(args.config)
    radius_cap = 2.5 if TAG_RADIUS_CAP in config_text or TAG_EXACT_RADIUS_CAP in config_text else None

    candidates = load_candidates(args.candidate_csv)
    full_points = load_point_cloud(args.full_cloud_pcd)

    output_csv = args.output_csv or args.candidate_csv.with_name(
        args.candidate_csv.stem + "_fullcloud_checked.csv"
    )

    kept = []
    rejected = []
    for candidate in candidates:
        if radius_cap is not None and candidate["radius"] > radius_cap:
            rejected.append((candidate, -1))
            continue
        accepted, count_above = check_candidate(candidate, full_points, args.threshold, args.max_points_above)
        if accepted:
            kept.append(candidate)
        else:
            rejected.append((candidate, count_above))

    write_filtered_csv(kept, output_csv)

    print(f"Loaded {len(candidates)} candidate circles from: {args.candidate_csv}")
    print(f"Loaded {len(full_points)} points from full cloud: {args.full_cloud_pcd}")
    print(f"Accepted {len(kept)} circles, rejected {len(rejected)} circles.")
    print(f"Wrote filtered landing zones to: {output_csv}")

    if rejected:
        print("Rejected circles due to full-cloud obstacle count:")
        for candidate, count_above in rejected[:20]:
            print(
                f"  rank={int(candidate['rank'])}, center=({candidate['x']:.3f},{candidate['y']:.3f},{candidate['z']:.3f}), "
                f"radius={candidate['radius']:.3f}, points_above={count_above}"
            )
        if len(rejected) > 20:
            print(f"  ...and {len(rejected) - 20} more rejected circles.")

    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(1)
