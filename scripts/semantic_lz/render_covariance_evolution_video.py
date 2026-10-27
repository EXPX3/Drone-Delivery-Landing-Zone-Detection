#!/usr/bin/env python3
"""Render 2D/3D semantic localization and covariance evolution as MP4."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Ellipse

from render_object_orthophoto import (
    GROUND_TRUTH_LOCAL_XY, GROUND_TRUTH_ORIGIN_UTM, GROUND_TRUTH_ROTATION,
    PCDS, read_pcd_bounds,
)


COLORS = {"cargo_box": "#ffe100", "person": "#ff00dc"}
THRESHOLDS = {
    "cargo_box": {"confidence": 0.10, "mask": 0.08, "points": 2,
                  "radial": 0.75, "reference_points": 10},
    "person": {"confidence": 0.30, "mask": 0.05, "points": 2,
               "radial": 0.80, "reference_points": 5},
}


def covariance(points: np.ndarray) -> np.ndarray:
    return np.cov(points[:, :2], rowvar=False) if len(points) > 1 else np.eye(2) * 0.01


def ellipse_parameters(matrix: np.ndarray, scale: float = np.sqrt(5.991)) -> tuple[float, float, float]:
    values, vectors = np.linalg.eigh(matrix)
    order = np.argsort(values)[::-1]
    values, vectors = values[order], vectors[:, order]
    width, height = 2.0 * scale * np.sqrt(np.maximum(values, 0.0))
    angle = np.degrees(np.arctan2(vectors[1, 0], vectors[0, 0]))
    return float(width), float(height), float(angle)


def accepted_observations(raw: list[dict], targets: list[dict], association_m: float) -> None:
    for item in sorted(raw, key=lambda value: value["relative_time_s"]):
        class_name = item["class_name"]
        if class_name not in THRESHOLDS:
            continue
        rule = THRESHOLDS[class_name]
        support = item["height_verified"]
        if (item["confidence"] < rule["confidence"]
                or item["mask_fraction"] > rule["mask"]
                or support["count"] < rule["points"]
                or support.get("radial_p95_m", np.inf) > rule["radial"]):
            continue
        point = np.asarray(support["median_xyz"], dtype=np.float64)
        candidates = [target for target in targets if target["class_name"] == class_name]
        if not candidates:
            continue
        distances = [np.linalg.norm(point[:2] - target["final_center"][:2]) for target in candidates]
        nearest = int(np.argmin(distances))
        if distances[nearest] <= association_m:
            support_score = 1.0 - np.exp(-support["count"] / rule["reference_points"])
            compactness_score = np.exp(
                -0.5 * (support["radial_p95_m"] / rule["radial"]) ** 2
            )
            lidar_score = float(np.sqrt(support_score * compactness_score))
            camera_score = float(item["confidence"])
            fused_score = float(np.sqrt(camera_score * lidar_score))
            candidates[nearest]["observations"].append(
                (float(item["relative_time_s"]), point,
                 camera_score, lidar_score, fused_score)
            )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--audit", type=Path,
                        default=PCDS / "semantic_experiment/full_spread_audit.json")
    parser.add_argument("--validated", type=Path,
                        default=PCDS / "semantic_experiment/retained_tracks_validation.json")
    parser.add_argument("--orthophoto", type=Path, default=PCDS /
                        "696_5328_cropped_by_bayernatlas_20260812084517_satellite_overlay_dop20.png")
    parser.add_argument("--pcd", type=Path, default=PCDS /
                        "696_5328_cropped_by_bayernatlas_20260812084517.pcd")
    parser.add_argument("--output", type=Path, default=PCDS / "semantic_experiment" /
                        "semantic_localization_covariance_evolution.mp4")
    parser.add_argument("--playback-speed", type=float, default=10.0)
    parser.add_argument("--fps", type=float, default=12.0)
    parser.add_argument("--association-m", type=float, default=1.2)
    parser.add_argument("--duration-s", type=float, default=346.734656736,
                        help="Full bag duration; keep the final stable state after the last detection")
    parser.add_argument("--gaussian-display-scale", type=float, default=4.0,
                        help="Horizontal-only visual exaggeration for readable 3D Gaussian surfaces")
    args = parser.parse_args()

    audit = json.loads(args.audit.read_text(encoding="utf-8"))
    validated = json.loads(args.validated.read_text(encoding="utf-8"))
    confirmed = [track for track in validated["guided_lidar_tracks"] if track["confirmed"]]
    cargo = sorted((track for track in confirmed if track["class_name"] == "cargo_box"),
                   key=lambda track: track["track_id"])
    people = sorted((track for track in confirmed if track["class_name"] == "person"),
                    key=lambda track: track["track_id"])
    targets = []
    for index, track in enumerate(cargo, 1):
        targets.append({"name": f"Cargo {index}", "class_name": "cargo_box",
                        "final_center": np.asarray(track["centroid"]), "observations": []})
    for index, track in enumerate(people, 1):
        targets.append({"name": f"Person {index}", "class_name": "person",
                        "final_center": np.asarray(track["centroid"]), "observations": []})
    accepted_observations(audit["semantic_observations"], targets, args.association_m)
    ground_truth = {
        name: GROUND_TRUTH_ROTATION @ local + GROUND_TRUTH_ORIGIN_UTM
        for name, local in GROUND_TRUTH_LOCAL_XY.items()
    }

    duration = float(args.duration_s)
    frame_count = int(np.ceil(duration / args.playback_speed * args.fps)) + 1
    source_bounds = read_pcd_bounds(args.pcd)
    image = plt.imread(args.orthophoto)
    final_centers = np.asarray([target["final_center"][:2] for target in targets])
    context = 12.0
    view = (float(final_centers[:, 0].min() - context),
            float(final_centers[:, 0].max() + context),
            float(final_centers[:, 1].min() - context),
            float(final_centers[:, 1].max() + context))
    local_origin = np.median(final_centers, axis=0)

    width, height = 1600, 900
    args.output.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(str(args.output), cv2.VideoWriter_fourcc(*"mp4v"),
                             args.fps, (width, height))
    if not writer.isOpened():
        raise RuntimeError("OpenCV could not initialize the MP4 writer")

    plt.style.use("dark_background")
    fig = plt.figure(figsize=(16, 9), dpi=100)
    grid = fig.add_gridspec(2, 2, width_ratios=(1.45, 1.0), height_ratios=(1.12, 0.88),
                           left=0.035, right=0.98, top=0.91, bottom=0.06,
                           wspace=0.16, hspace=0.22)
    ax_map = fig.add_subplot(grid[:, 0])
    ax_3d = fig.add_subplot(grid[0, 1], projection="3d")
    ax_metrics = fig.add_subplot(grid[1, 1])

    for frame_index in range(frame_count):
        bag_time = min(duration, frame_index / args.fps * args.playback_speed)
        ax_map.clear()
        ax_3d.clear()
        ax_metrics.clear()
        min_x, min_y, max_x, max_y = source_bounds
        ax_map.imshow(image, extent=(min_x, max_x, min_y, max_y), origin="upper")
        ax_map.set_xlim(view[0], view[1])
        ax_map.set_ylim(view[2], view[3])
        ax_map.set_aspect("equal")
        ax_map.set_title("2D orthophoto localization and 95% covariance", fontsize=15, weight="bold")
        ax_map.set_xlabel("UTM Easting [m]")
        ax_map.set_ylabel("UTM Northing [m]")
        ax_map.grid(alpha=0.18)

        # Static survey reference remains visible for the complete video. It is
        # deliberately subdued so evolving detections and covariance stay primary.
        for name, truth in ground_truth.items():
            ax_map.scatter(truth[0], truth[1], marker="x", s=95, c="#00ebff",
                           linewidths=2.2, alpha=.55, zorder=3)
            ax_map.annotate(f"GT {name}", truth, xytext=(-6, -13),
                            textcoords="offset points", color="#8ff7ff", fontsize=7.5,
                            alpha=.70, ha="right", zorder=3)
        gt_order = ["Cargo 3", "Cargo 1", "Cargo 2"]
        gt_line = np.asarray([ground_truth[name] for name in gt_order])
        ax_map.plot(gt_line[:, 0], gt_line[:, 1], color="#00ebff", ls="--",
                    lw=1.1, alpha=.20, zorder=2)

        active = []
        for target in targets:
            observations = [item for item in target["observations"] if item[0] <= bag_time]
            if not observations:
                continue
            values = [item[1] for item in observations]
            points = np.asarray(values)
            center = np.median(points, axis=0)
            matrix = covariance(points)
            confirmed_now = len(points) >= 3
            color = COLORS[target["class_name"]] if confirmed_now else "#b7bec9"
            alpha = 0.95 if confirmed_now else 0.55
            width_e, height_e, angle = ellipse_parameters(matrix)
            ax_map.add_patch(Ellipse(center[:2], width_e, height_e, angle=angle,
                                     facecolor=color, edgecolor=color, alpha=0.18,
                                     linewidth=2.5, zorder=4))
            ax_map.scatter(center[0], center[1], s=110, c=color, edgecolors="black",
                           linewidths=1.4, zorder=6, alpha=alpha)
            truth = ground_truth[target["name"]]
            ax_map.annotate("", xy=center[:2], xytext=truth,
                            arrowprops=dict(arrowstyle="->", color="#00ebff",
                                            lw=1.2, alpha=.48), zorder=5)
            status = "confirmed" if confirmed_now else f"provisional {len(points)}/3"
            ax_map.annotate(f"{target['name']}\n{status}", center[:2], xytext=(8, 8),
                            textcoords="offset points", color=color, fontsize=9,
                            weight="bold", zorder=7,
                            bbox=dict(boxstyle="round,pad=.25", fc="#111722", ec=color, alpha=.86))
            if confirmed_now:
                scores = np.asarray([item[2:] for item in observations], dtype=np.float64)
                active.append((target, points, center, matrix, color, scores))

        for first_index in range(len(active)):
            for second_index in range(first_index + 1, len(active)):
                first, second = active[first_index], active[second_index]
                a, b = first[2][:2], second[2][:2]
                distance = np.linalg.norm(b - a)
                ax_map.plot((a[0], b[0]), (a[1], b[1]), color="white", alpha=.34, lw=1)
                middle = (a + b) * .5
                ax_map.text(middle[0], middle[1], f"{distance:.2f} m", fontsize=7,
                            ha="center", va="center", color="white",
                            bbox=dict(boxstyle="round,pad=.18", fc="#10151d", ec="none", alpha=.75))

        ax_3d.set_title(
            f"3D evolving Gaussian surfaces ({args.gaussian_display_scale:g}x XY display scale)",
            fontsize=14, weight="bold",
        )
        ax_3d.set_xlabel("East relative [m]")
        ax_3d.set_ylabel("North relative [m]")
        ax_3d.set_zlabel("relative density")
        ax_3d.set_xlim(view[0] - local_origin[0], view[1] - local_origin[0])
        ax_3d.set_ylim(view[2] - local_origin[1], view[3] - local_origin[1])
        ax_3d.set_zlim(0.0, 1.1)
        ax_3d.view_init(elev=34, azim=-58)
        for name, truth in ground_truth.items():
            gx, gy = truth - local_origin
            ax_3d.scatter(gx, gy, 0.0, marker="x", s=48, c="#00ebff",
                          linewidths=1.8, alpha=.60, depthshade=False)
            ax_3d.text(gx, gy, 0.015, f"GT {name}\nXY only", color="#8ff7ff",
                       alpha=.70, fontsize=6, ha="center", va="bottom")
        gt_relative = gt_line - local_origin
        ax_3d.plot(gt_relative[:, 0], gt_relative[:, 1], np.zeros(len(gt_relative)),
                   color="#00ebff", ls="--", lw=1.0, alpha=.18)
        for target, points, center, matrix, color, _scores in active:
            values, vectors = np.linalg.eigh(matrix)
            values = np.maximum(values, 0.0025)
            u = np.linspace(-3.0, 3.0, 28)
            v = np.linspace(-3.0, 3.0, 28)
            uu, vv = np.meshgrid(u, v)
            offsets = np.stack((uu * np.sqrt(values[0]), vv * np.sqrt(values[1])), axis=-1)
            rotated = (offsets @ vectors.T) * args.gaussian_display_scale
            xx = rotated[:, :, 0] + center[0] - local_origin[0]
            yy = rotated[:, :, 1] + center[1] - local_origin[1]
            zz = np.exp(-0.5 * (uu * uu + vv * vv))
            ax_3d.plot_surface(xx, yy, zz, color=color, linewidth=0.0,
                               antialiased=True, alpha=.30, shade=True)
            ax_3d.plot_wireframe(xx, yy, zz, rstride=3, cstride=3, color=color,
                                 linewidth=.55, alpha=.72)
            ax_3d.text(center[0] - local_origin[0], center[1] - local_origin[1], 1.04,
                       target["name"], color=color, fontsize=8, ha="center")

        ax_metrics.axis("off")
        ax_metrics.text(0.0, 1.02, "Live localization metrics", fontsize=15,
                        weight="bold", transform=ax_metrics.transAxes)
        y = 0.92
        if not active:
            ax_metrics.text(0.0, y, "Waiting for three consistent observations...",
                            color="#b7bec9", transform=ax_metrics.transAxes)
        for target, points, center, matrix, color, scores in active:
            radial = np.linalg.norm(points[:, :2] - np.median(points[:, :2], axis=0), axis=1)
            p95 = np.percentile(radial, 95)
            camera_score, lidar_score, fused_score = np.median(scores, axis=0)
            truth = ground_truth[target["name"]]
            residual = center[:2] - truth
            error = np.linalg.norm(residual)
            local = GROUND_TRUTH_LOCAL_XY[target["name"]]
            text = (f"{target['name']} | n={len(points)} | P95={p95:.3f} m\n"
                    f"Ccam={camera_score:.3f}  Clidar={lidar_score:.3f}  Cfused={fused_score:.3f}\n"
                    f"E={center[0]:.3f}, N={center[1]:.3f}, Z={center[2]:.3f} m\n"
                    f"GTxy=({local[0]:.2f},{local[1]:.2f}) err={error:.3f} m; GT Z unmeasured\n"
                    f"Cov XY [m²]  [{matrix[0, 0]:.4f}  {matrix[0, 1]:+.4f}]\n"
                    f"             [{matrix[1, 0]:+.4f}  {matrix[1, 1]:.4f}]")
            ax_metrics.text(0.0, y, text, color=color, family="monospace", fontsize=7.6,
                            va="top", transform=ax_metrics.transAxes)
            y -= 0.245
        fig.suptitle(f"Semantic object localization uncertainty | bag t={bag_time:6.1f}/{duration:.1f} s | "
                     f"{args.playback_speed:g}x", fontsize=18, weight="bold")
        fig.canvas.draw()
        rgb = np.asarray(fig.canvas.buffer_rgba())[:, :, :3]
        writer.write(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))

    writer.release()
    plt.close(fig)
    print(f"Wrote {args.output} ({frame_count} frames, {frame_count / args.fps:.1f} s)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
