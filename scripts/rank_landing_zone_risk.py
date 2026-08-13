#!/usr/bin/env python3
"""Rank landing-zone candidates using orthophoto and point-cloud risk evidence.

The script produces auditable risk metrics for each candidate landing zone:

* orthophoto vegetation/grass evidence from color indices and texture
* point-cloud relief, roughness, point density, vertical obstacle count
* offset to nearby vertical obstacles, including a GPS/wind safety margin
* optional Segment Anything mask support when a real SAM checkpoint is supplied

Outputs are written next to the candidate CSV by default:

* ranked CSV with safest/safe/risky labels
* orthophoto risk and obstacle PNG layers
* Plotly HTML overlay with green/orange/red candidate circles
* TeX report describing the methods and run results
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import cv2
import numpy as np
import plotly.graph_objects as go
from PIL import Image
from scipy import ndimage
from scipy.spatial import cKDTree
from skimage import color, exposure, filters, morphology, segmentation, util


ROOT = Path(__file__).resolve().parents[1]
PCDS = ROOT / "pcds"


@dataclass
class Candidate:
    rank: int
    x: float
    y: float
    z: float
    radius: float
    normal_x: float
    normal_y: float
    normal_z: float


@dataclass
class Bounds:
    minx: float
    miny: float
    maxx: float
    maxy: float


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--candidates", type=Path, default=PCDS / "region_growing_landing_zones_max10000_radius2_5_to3.csv")
    parser.add_argument("--pcd", type=Path, default=PCDS / "696_5328_cropped_by_bayernatlas_20260812084517.pcd")
    parser.add_argument("--classified-pcd", type=Path, default=PCDS / "696_5328_cropped_by_bayernatlas_20260812084517_classified.pcd")
    parser.add_argument("--orthophoto", type=Path, default=PCDS / "696_5328_cropped_by_bayernatlas_20260812084517_satellite_overlay_dop20.png")
    parser.add_argument("--output-prefix", type=Path, default=None)
    parser.add_argument("--html-max-points", type=int, default=180000)
    parser.add_argument("--orthophoto-stride", type=int, default=4)
    parser.add_argument("--gps-error-m", type=float, default=1.0)
    parser.add_argument("--wind-speed-mps", type=float, default=4.0)
    parser.add_argument("--descent-speed-mps", type=float, default=1.0)
    parser.add_argument("--wind-factor", type=float, default=0.25, help="Meters of horizontal drift per wind/descent speed ratio.")
    parser.add_argument("--obstacle-height-m", type=float, default=0.7)
    parser.add_argument("--obstacle-search-m", type=float, default=8.0)
    parser.add_argument("--safest-threshold", type=float, default=0.33)
    parser.add_argument("--safe-threshold", type=float, default=0.62)
    parser.add_argument("--sam-checkpoint", type=Path, default=None, help="Optional real SAM checkpoint. If absent, SAM is not used.")
    parser.add_argument("--sam-model-type", default="vit_h")
    return parser.parse_args()


def read_candidates(path: Path) -> list[Candidate]:
    out: list[Candidate] = []
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            out.append(
                Candidate(
                    rank=int(float(row["rank"])),
                    x=float(row["x"]),
                    y=float(row["y"]),
                    z=float(row["z"]),
                    radius=float(row["radius"]),
                    normal_x=float(row["normal_x"]),
                    normal_y=float(row["normal_y"]),
                    normal_z=float(row["normal_z"]),
                )
            )
    if not out:
        raise RuntimeError(f"No candidates found in {path}")
    return out


def read_ascii_pcd(path: Path, require_classification: bool = False) -> tuple[np.ndarray, np.ndarray | None]:
    fields: list[str] | None = None
    data_start = None
    with path.open("r", encoding="utf-8") as handle:
        for line_no, line in enumerate(handle):
            line = line.strip()
            if line.startswith("FIELDS"):
                fields = line.split()[1:]
            if line.startswith("DATA"):
                if "ascii" not in line:
                    raise RuntimeError(f"Only ASCII PCD is supported by this script: {path}")
                data_start = line_no + 1
                break
    if fields is None or data_start is None:
        raise RuntimeError(f"Invalid PCD header: {path}")
    arr = np.loadtxt(path, skiprows=data_start, dtype=np.float64)
    if arr.ndim == 1:
        arr = arr.reshape(1, -1)
    field_index = {name: idx for idx, name in enumerate(fields)}
    xyz = arr[:, [field_index["x"], field_index["y"], field_index["z"]]]
    classification = None
    if "classification" in field_index:
        classification = arr[:, field_index["classification"]].astype(np.int32)
    elif require_classification:
        raise RuntimeError(f"PCD has no classification field: {path}")
    return xyz, classification


def pcd_bounds(points: np.ndarray, margin: float = 10.0) -> Bounds:
    return Bounds(
        minx=float(np.min(points[:, 0]) - margin),
        miny=float(np.min(points[:, 1]) - margin),
        maxx=float(np.max(points[:, 0]) + margin),
        maxy=float(np.max(points[:, 1]) + margin),
    )


def world_to_pixel(x: np.ndarray | float, y: np.ndarray | float, bounds: Bounds, width: int, height: int) -> tuple[np.ndarray, np.ndarray]:
    xx = np.asarray(x, dtype=np.float64)
    yy = np.asarray(y, dtype=np.float64)
    col = (xx - bounds.minx) / (bounds.maxx - bounds.minx) * (width - 1)
    row = (bounds.maxy - yy) / (bounds.maxy - bounds.miny) * (height - 1)
    return row, col


def pixel_to_world(rows: np.ndarray, cols: np.ndarray, bounds: Bounds, width: int, height: int) -> tuple[np.ndarray, np.ndarray]:
    x = bounds.minx + cols / max(1, width - 1) * (bounds.maxx - bounds.minx)
    y = bounds.maxy - rows / max(1, height - 1) * (bounds.maxy - bounds.miny)
    return x, y


def robust01(values: np.ndarray, lo: float | None = None, hi: float | None = None) -> np.ndarray:
    values = np.asarray(values, dtype=np.float64)
    if lo is None:
        lo = float(np.nanpercentile(values, 2))
    if hi is None:
        hi = float(np.nanpercentile(values, 98))
    if hi <= lo:
        return np.zeros_like(values)
    return np.clip((values - lo) / (hi - lo), 0.0, 1.0)


def make_orthophoto_layers(image_path: Path) -> dict[str, np.ndarray]:
    rgba = Image.open(image_path).convert("RGBA")
    rgb = np.asarray(rgba)[..., :3].astype(np.float32) / 255.0
    lab = color.rgb2lab(rgb)
    hsv = color.rgb2hsv(rgb)
    gray = color.rgb2gray(rgb)
    texture = filters.rank.entropy(util.img_as_ubyte(gray), morphology.disk(5)).astype(np.float32)
    texture01 = robust01(texture)

    r, g, b = rgb[..., 0], rgb[..., 1], rgb[..., 2]
    exg = 2.0 * g - r - b
    vari = (g - r) / (g + r - b + 1e-6)
    green_score = 0.45 * robust01(exg) + 0.35 * robust01(vari) + 0.20 * robust01(hsv[..., 1])
    dark_score = 1.0 - robust01(gray)
    tree_score = np.clip(0.58 * green_score + 0.28 * texture01 + 0.14 * dark_score, 0.0, 1.0)
    grass_score = np.clip(0.72 * green_score + 0.18 * (1.0 - texture01) + 0.10 * robust01(lab[..., 0]), 0.0, 1.0)

    # Superpixels reduce speckle while keeping edges from the orthophoto.
    labels = segmentation.slic(rgb, n_segments=850, compactness=12.0, sigma=1.0, start_label=1)
    for score_name in ("tree_score", "grass_score"):
        score = tree_score if score_name == "tree_score" else grass_score
        mean = ndimage.mean(score, labels=labels, index=np.arange(1, labels.max() + 1))
        smoothed = mean[labels - 1]
        if score_name == "tree_score":
            tree_score = smoothed
        else:
            grass_score = smoothed

    tree_mask = tree_score >= np.nanpercentile(tree_score, 72)
    tree_mask = morphology.remove_small_objects(tree_mask, max_size=47)
    tree_mask = morphology.closing(tree_mask, morphology.disk(3))
    grass_mask = (grass_score >= np.nanpercentile(grass_score, 58)) & ~tree_mask
    grass_mask = morphology.remove_small_objects(grass_mask, max_size=47)

    obstacle_distance_px = ndimage.distance_transform_edt(~tree_mask)
    return {
        "rgb": rgb,
        "tree_score": tree_score.astype(np.float32),
        "grass_score": grass_score.astype(np.float32),
        "tree_mask": tree_mask,
        "grass_mask": grass_mask,
        "obstacle_distance_px": obstacle_distance_px.astype(np.float32),
        "texture": texture01.astype(np.float32),
    }


def maybe_sam_tree_layer(args: argparse.Namespace, image_rgb: np.ndarray) -> np.ndarray | None:
    if args.sam_checkpoint is None:
        return None
    if not args.sam_checkpoint.exists():
        raise FileNotFoundError(f"SAM checkpoint not found: {args.sam_checkpoint}")
    try:
        from segment_anything import SamAutomaticMaskGenerator, sam_model_registry
    except ImportError as exc:
        raise RuntimeError(
            "segment_anything is not installed. Install the real Meta AI SAM package or omit --sam-checkpoint."
        ) from exc

    sam = sam_model_registry[args.sam_model_type](checkpoint=str(args.sam_checkpoint))
    generator = SamAutomaticMaskGenerator(
        sam,
        points_per_side=32,
        pred_iou_thresh=0.88,
        stability_score_thresh=0.92,
        min_mask_region_area=80,
    )
    masks = generator.generate((image_rgb * 255).astype(np.uint8))
    tree_layer = np.zeros(image_rgb.shape[:2], dtype=bool)
    layers = make_orthophoto_layers_from_rgb(image_rgb)
    for mask in masks:
        seg = mask["segmentation"]
        if np.mean(layers["tree_score"][seg]) > 0.58:
            tree_layer |= seg
    return morphology.closing(tree_layer, morphology.disk(3))


def make_orthophoto_layers_from_rgb(rgb: np.ndarray) -> dict[str, np.ndarray]:
    tmp = Path("/tmp/ddlzd_tmp_orthophoto.png")
    Image.fromarray((np.clip(rgb, 0, 1) * 255).astype(np.uint8)).save(tmp)
    return make_orthophoto_layers(tmp)


def save_layer_png(values: np.ndarray, output: Path, cmap: int = cv2.COLORMAP_TURBO) -> None:
    arr = (robust01(values) * 255).astype(np.uint8)
    colored = cv2.applyColorMap(arr, cmap)
    cv2.imwrite(str(output), colored)


def circular_pixel_mask(cx: float, cy: float, radius_m: float, bounds: Bounds, width: int, height: int) -> tuple[np.ndarray, np.ndarray]:
    row, col = world_to_pixel(cx, cy, bounds, width, height)
    meters_per_px_x = (bounds.maxx - bounds.minx) / max(1, width - 1)
    meters_per_px_y = (bounds.maxy - bounds.miny) / max(1, height - 1)
    radius_px = radius_m / ((meters_per_px_x + meters_per_px_y) * 0.5)
    r0 = max(0, int(math.floor(float(row) - radius_px)))
    r1 = min(height, int(math.ceil(float(row) + radius_px + 1)))
    c0 = max(0, int(math.floor(float(col) - radius_px)))
    c1 = min(width, int(math.ceil(float(col) + radius_px + 1)))
    rr, cc = np.meshgrid(np.arange(r0, r1), np.arange(c0, c1), indexing="ij")
    mask = (rr - float(row)) ** 2 + (cc - float(col)) ** 2 <= radius_px**2
    return rr[mask], cc[mask]


def compute_metrics(
    candidates: list[Candidate],
    points: np.ndarray,
    classes: np.ndarray | None,
    layers: dict[str, np.ndarray],
    bounds: Bounds,
    args: argparse.Namespace,
) -> list[dict[str, float | int | str]]:
    height, width = layers["tree_score"].shape
    tree_xy = cKDTree(points[:, :2])
    high_points = points[points[:, 2] >= np.percentile(points[:, 2], 75)]
    high_tree = cKDTree(high_points[:, :2]) if len(high_points) else None
    drift_margin = args.gps_error_m + args.wind_factor * args.wind_speed_mps / max(args.descent_speed_mps, 1e-6)
    effective_margin = drift_margin + 0.25

    rows: list[dict[str, float | int | str]] = []
    for cand in candidates:
        center = np.array([cand.x, cand.y, cand.z], dtype=np.float64)
        radius = cand.radius
        query_radius = radius + effective_margin
        point_ids = tree_xy.query_ball_point([cand.x, cand.y], query_radius)
        local = points[point_ids] if point_ids else np.empty((0, 3))
        if len(local):
            dz = local[:, 2] - cand.z
            in_disc = np.linalg.norm(local[:, :2] - center[:2], axis=1) <= query_radius
            disc = local[in_disc]
            disc_dz = disc[:, 2] - cand.z if len(disc) else np.array([])
            obstacle_count = int(np.count_nonzero(disc_dz > args.obstacle_height_m))
            relief = float(np.percentile(disc[:, 2], 95) - np.percentile(disc[:, 2], 5)) if len(disc) else 0.0
            roughness = float(np.std(disc_dz)) if len(disc_dz) else 0.0
            density = float(len(disc) / (math.pi * query_radius * query_radius)) if query_radius > 0 else 0.0
        else:
            obstacle_count, relief, roughness, density = 0, 0.0, 0.0, 0.0

        if high_tree is not None:
            dist, _idx = high_tree.query([cand.x, cand.y], k=1)
            nearest_high = float(dist)
        else:
            nearest_high = float("inf")

        rr, cc = circular_pixel_mask(cand.x, cand.y, query_radius, bounds, width, height)
        if len(rr):
            tree_frac = float(np.mean(layers["tree_mask"][rr, cc]))
            grass_frac = float(np.mean(layers["grass_mask"][rr, cc]))
            mean_tree_score = float(np.mean(layers["tree_score"][rr, cc]))
            mean_grass_score = float(np.mean(layers["grass_score"][rr, cc]))
            mean_texture = float(np.mean(layers["texture"][rr, cc]))
            obstacle_px = float(np.min(layers["obstacle_distance_px"][rr, cc]))
            meters_per_px = ((bounds.maxx - bounds.minx) / max(1, width - 1) + (bounds.maxy - bounds.miny) / max(1, height - 1)) * 0.5
            ortho_obstacle_offset = obstacle_px * meters_per_px
        else:
            tree_frac = grass_frac = mean_tree_score = mean_grass_score = mean_texture = 0.0
            ortho_obstacle_offset = 0.0

        class20_frac = 0.0
        if classes is not None and point_ids:
            local_classes = classes[point_ids]
            class20_frac = float(np.mean(local_classes == 20))

        slope_deg = math.degrees(math.acos(np.clip(abs(cand.normal_z), 0.0, 1.0)))
        obstacle_offset = min(nearest_high, ortho_obstacle_offset)

        rows.append(
            {
                "rank_original": cand.rank,
                "x": cand.x,
                "y": cand.y,
                "z": cand.z,
                "radius": radius,
                "normal_x": cand.normal_x,
                "normal_y": cand.normal_y,
                "normal_z": cand.normal_z,
                "slope_deg": slope_deg,
                "pc_density_pts_m2": density,
                "pc_relief_m": relief,
                "pc_roughness_m": roughness,
                "pc_obstacle_count": obstacle_count,
                "pc_nearest_high_point_m": nearest_high,
                "ortho_tree_fraction": tree_frac,
                "ortho_grass_fraction": grass_frac,
                "ortho_tree_score": mean_tree_score,
                "ortho_grass_score": mean_grass_score,
                "ortho_texture": mean_texture,
                "ortho_obstacle_offset_m": ortho_obstacle_offset,
                "las_class20_fraction": class20_frac,
                "gps_wind_margin_m": drift_margin,
                "obstacle_offset_m": obstacle_offset,
            }
        )
    return rows


def score_rows(rows: list[dict[str, float | int | str]], safest_threshold: float, safe_threshold: float) -> None:
    relief = np.array([float(r["pc_relief_m"]) for r in rows])
    rough = np.array([float(r["pc_roughness_m"]) for r in rows])
    obs = np.array([float(r["pc_obstacle_count"]) for r in rows])
    tree = np.array([float(r["ortho_tree_fraction"]) for r in rows])
    tree_score = np.array([float(r["ortho_tree_score"]) for r in rows])
    grass = np.array([float(r["ortho_grass_fraction"]) for r in rows])
    texture = np.array([float(r["ortho_texture"]) for r in rows])
    slope = np.array([float(r["slope_deg"]) for r in rows])
    offset = np.array([float(r["obstacle_offset_m"]) for r in rows])
    class20 = np.array([float(r["las_class20_fraction"]) for r in rows])

    relief_r = robust01(relief, 0.1, 4.5)
    rough_r = robust01(rough, 0.03, 0.55)
    obs_r = robust01(obs, 0.0, 80.0)
    tree_r = np.clip(0.55 * tree + 0.45 * tree_score, 0.0, 1.0)
    grass_bonus = np.clip(grass, 0.0, 1.0)
    texture_r = np.clip(texture, 0.0, 1.0)
    slope_r = robust01(slope, 0.0, 8.0)
    offset_r = 1.0 - robust01(offset, 2.0, 12.0)
    class20_r = np.clip(class20, 0.0, 1.0)

    for idx, row in enumerate(rows):
        risk = (
            0.23 * obs_r[idx]
            + 0.18 * offset_r[idx]
            + 0.17 * tree_r[idx]
            + 0.14 * relief_r[idx]
            + 0.10 * rough_r[idx]
            + 0.07 * texture_r[idx]
            + 0.06 * slope_r[idx]
            + 0.05 * class20_r[idx]
            - 0.10 * grass_bonus[idx]
        )
        risk = float(np.clip(risk, 0.0, 1.0))
        row["risk_score"] = risk
        if risk <= safest_threshold:
            row["risk_category"] = "safest"
        elif risk <= safe_threshold:
            row["risk_category"] = "safe"
        else:
            row["risk_category"] = "risky"

    rows.sort(key=lambda r: (float(r["risk_score"]), -float(r["radius"]), -float(r["ortho_grass_fraction"])))
    for idx, row in enumerate(rows, start=1):
        row["rank_risk"] = idx


def write_ranked_csv(rows: list[dict[str, float | int | str]], path: Path) -> None:
    fields = [
        "rank_risk",
        "risk_category",
        "risk_score",
        "rank_original",
        "x",
        "y",
        "z",
        "radius",
        "normal_x",
        "normal_y",
        "normal_z",
        "slope_deg",
        "pc_density_pts_m2",
        "pc_relief_m",
        "pc_roughness_m",
        "pc_obstacle_count",
        "pc_nearest_high_point_m",
        "ortho_tree_fraction",
        "ortho_grass_fraction",
        "ortho_tree_score",
        "ortho_grass_score",
        "ortho_texture",
        "ortho_obstacle_offset_m",
        "las_class20_fraction",
        "gps_wind_margin_m",
        "obstacle_offset_m",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def make_html(
    rows: list[dict[str, float | int | str]],
    points: np.ndarray,
    image_path: Path,
    bounds: Bounds,
    output: Path,
    max_points: int,
    stride: int,
) -> None:
    minz = float(np.min(points[:, 2]))
    if len(points) > max_points:
        rng = np.random.default_rng(7)
        keep = np.sort(rng.choice(len(points), size=max_points, replace=False))
        plot_points = points[keep]
    else:
        plot_points = points

    fig = go.Figure()
    image = np.asarray(Image.open(image_path).convert("RGB"))[::stride, ::stride]
    h, w, _ = image.shape
    xs = np.linspace(bounds.minx, bounds.maxx, w)
    ys = np.linspace(bounds.maxy, bounds.miny, h)
    xx, yy = np.meshgrid(xs, ys)
    colors = [f"rgb({r},{g},{b})" for r, g, b in image.reshape(-1, 3)]
    fig.add_trace(
        go.Scatter3d(
            x=xx.ravel(),
            y=yy.ravel(),
            z=np.full(xx.size, minz - 0.5),
            mode="markers",
            name="Bayern DOP20 orthophoto",
            marker={"size": 2.0, "color": colors, "opacity": 1.0},
            hoverinfo="skip",
        )
    )
    fig.add_trace(
        go.Scatter3d(
            x=plot_points[:, 0],
            y=plot_points[:, 1],
            z=plot_points[:, 2],
            mode="markers",
            name="Point cloud",
            marker={"size": 1.2, "color": plot_points[:, 2], "colorscale": "Viridis", "opacity": 0.55},
        )
    )

    colors_by_category = {"safest": "green", "safe": "orange", "risky": "red"}
    for category in ("safest", "safe", "risky"):
        category_rows = [r for r in rows if r["risk_category"] == category]
        for row in category_rows:
            theta = np.linspace(0, 2 * np.pi, 96)
            cx, cy, cz, radius = float(row["x"]), float(row["y"]), float(row["z"]), float(row["radius"])
            hover = (
                f"risk rank={row['rank_risk']}<br>"
                f"category={category}<br>"
                f"score={float(row['risk_score']):.3f}<br>"
                f"grass={float(row['ortho_grass_fraction']):.2f}<br>"
                f"tree={float(row['ortho_tree_fraction']):.2f}<br>"
                f"obs_count={int(row['pc_obstacle_count'])}<br>"
                f"offset={float(row['obstacle_offset_m']):.2f} m"
            )
            fig.add_trace(
                go.Scatter3d(
                    x=cx + radius * np.cos(theta),
                    y=cy + radius * np.sin(theta),
                    z=np.full_like(theta, cz + 0.35),
                    mode="lines",
                    name=f"{category} LZ",
                    legendgroup=category,
                    showlegend=False,
                    line={"color": colors_by_category[category], "width": 7},
                    hovertext=hover,
                    hoverinfo="text",
                )
            )
        if category_rows:
            first = category_rows[0]
            fig.add_trace(
                go.Scatter3d(
                    x=[float(first["x"])],
                    y=[float(first["y"])],
                    z=[float(first["z"]) + 1.2],
                    mode="markers",
                    name=f"{category} ({len(category_rows)})",
                    legendgroup=category,
                    marker={"size": 8, "color": colors_by_category[category]},
                    hoverinfo="skip",
                )
            )

    top = rows[:30]
    for row in top:
        theta = np.linspace(0, 2 * np.pi, 96)
        cx, cy, cz, radius = float(row["x"]), float(row["y"]), float(row["z"]), float(row["radius"])
        fig.add_trace(
            go.Scatter3d(
                x=cx + radius * np.cos(theta),
                y=cy + radius * np.sin(theta),
                z=np.full_like(theta, cz + 0.7),
                mode="lines",
                name="Top 30 light-green circles",
                legendgroup="top30",
                showlegend=False,
                line={"color": "lightgreen", "width": 10},
                hovertext=(
                    f"top-30 risk rank={row['rank_risk']}<br>"
                    f"category={row['risk_category']}<br>"
                    f"score={float(row['risk_score']):.3f}"
                ),
                hoverinfo="text",
            )
        )
    if top:
        fig.add_trace(
            go.Scatter3d(
                x=[float(top[0]["x"])],
                y=[float(top[0]["y"])],
                z=[float(top[0]["z"]) + 1.6],
                mode="markers",
                name="Top 30 light green",
                legendgroup="top30",
                marker={"size": 9, "color": "lightgreen"},
                hoverinfo="skip",
            )
        )
    fig.add_trace(
        go.Scatter3d(
            x=[float(r["x"]) for r in top],
            y=[float(r["y"]) for r in top],
            z=[float(r["z"]) + 0.9 for r in top],
            mode="markers+text",
            name="Top 30 risk ranks",
            marker={"size": 4, "color": "black"},
            text=[str(r["rank_risk"]) for r in top],
            textposition="top center",
        )
    )
    fig.update_layout(
        title="Landing-zone risk ranking: green=safest, orange=safe, red=risky",
        scene={"xaxis_title": "UTM x", "yaxis_title": "UTM y", "zaxis_title": "z", "aspectmode": "data"},
        margin={"l": 0, "r": 0, "t": 35, "b": 0},
        legend={"title": {"text": "Layers"}, "x": 0.01, "y": 0.99, "bgcolor": "rgba(255,255,255,0.82)"},
    )
    fig.write_html(output, include_plotlyjs=True)


def write_tex_report(rows: list[dict[str, float | int | str]], args: argparse.Namespace, outputs: dict[str, Path], path: Path) -> None:
    counts = {cat: sum(1 for r in rows if r["risk_category"] == cat) for cat in ("safest", "safe", "risky")}
    top = rows[:10]
    mean_risk = float(np.mean([float(r["risk_score"]) for r in rows]))
    text = rf"""\documentclass[11pt]{{article}}
\usepackage[a4paper,margin=2.2cm]{{geometry}}
\usepackage{{booktabs}}
\usepackage{{hyperref}}
\title{{Landing-Zone Risk Ranking From Orthophoto and Point Cloud Evidence}}
\author{{DDLZD automated analysis}}
\date{{}}
\begin{{document}}
\maketitle

\section{{Objective}}
This report ranks the generated landing-zone candidates into safest, safe, and risky classes. The ranking combines orthophoto-derived semantic surface evidence with geometric point-cloud evidence and an explicit UAV positioning margin.

\section{{Inputs}}
\begin{{itemize}}
\item Candidate CSV: \texttt{{{args.candidates}}}
\item Point cloud: \texttt{{{args.pcd}}}
\item Classified point cloud: \texttt{{{args.classified_pcd}}}
\item Orthophoto: \texttt{{{args.orthophoto}}}
\end{{itemize}}

\section{{Method}}
The traditional computer-vision branch uses RGB color indices, HSV saturation, CIELab luminance, entropy texture, SLIC superpixels, and morphological filtering to estimate grass-like and tree-like image regions. The point-cloud branch computes local point density, height relief, roughness, vertical obstacle count above {args.obstacle_height_m:.2f} m, and nearest high-point offset. The UAV margin is
\[
m = e_{{GPS}} + k_w \frac{{v_w}}{{v_d}},
\]
with $e_{{GPS}}={args.gps_error_m:.2f}$ m, $v_w={args.wind_speed_mps:.2f}$ m/s, $v_d={args.descent_speed_mps:.2f}$ m/s, and $k_w={args.wind_factor:.2f}$.

The risk score is a weighted fusion of obstacle count, obstacle offset, orthophoto tree evidence, point-cloud relief and roughness, image texture, slope, LAS class-20 fraction, and a negative grass bonus. The thresholds used here are safest $\leq$ {args.safest_threshold:.2f}, safe $\leq$ {args.safe_threshold:.2f}, otherwise risky.

\section{{Deep Learning Variant}}
The script includes a real Segment Anything Model (SAM) adapter via \texttt{{--sam-checkpoint}}. It is only executed when the official \texttt{{segment\_anything}} package and a real checkpoint are supplied. No SAM checkpoint was used in this run, so the reported results are from the reproducible traditional CV plus point-cloud fusion branch.

\section{{Results}}
\begin{{itemize}}
\item Total candidates ranked: {len(rows)}
\item Safest: {counts['safest']}
\item Safe: {counts['safe']}
\item Risky: {counts['risky']}
\item Mean risk score: {mean_risk:.3f}
\end{{itemize}}

\begin{{table}}[h]
\centering
\begin{{tabular}}{{rrrrrrr}}
\toprule
Risk rank & Category & Score & X & Y & Radius & Obstacle offset\\
\midrule
"""
    for row in top:
        text += (
            f"{int(row['rank_risk'])} & {row['risk_category']} & {float(row['risk_score']):.3f} & "
            f"{float(row['x']):.2f} & {float(row['y']):.2f} & {float(row['radius']):.2f} & "
            f"{float(row['obstacle_offset_m']):.2f} \\\\\n"
        )
    text += rf"""\bottomrule
\end{{tabular}}
\caption{{Top 10 ranked landing-zone candidates.}}
\end{{table}}

\section{{Generated Artifacts}}
\begin{{itemize}}
\item Ranked CSV: \texttt{{{outputs['csv']}}}
\item HTML overlay: \texttt{{{outputs['html']}}}
\item Orthophoto risk layer: \texttt{{{outputs['risk_png']}}}
\item Orthophoto obstacle layer: \texttt{{{outputs['obstacle_png']}}}
\end{{itemize}}

\section{{Limitations}}
The orthophoto semantic layer is not a supervised land-cover classifier; it is an explainable zero-training segmentation tuned for this tile. The point-cloud contains ground and ignored-ground classes in the cropped Bayern data, so above-ground obstacle evidence is inferred from height structure rather than a rich object-class taxonomy. For publication-grade external validation, compare the labels against manual annotation or a checkpointed aerial land-cover network.

\end{{document}}
"""
    path.write_text(text, encoding="utf-8")


def main() -> int:
    args = parse_args()
    prefix = args.output_prefix or args.candidates.with_name(args.candidates.stem + "_risk_ranked")
    prefix.parent.mkdir(parents=True, exist_ok=True)

    candidates = read_candidates(args.candidates)
    points, _ = read_ascii_pcd(args.pcd)
    classes = None
    if args.classified_pcd.exists():
        class_points, classes = read_ascii_pcd(args.classified_pcd)
        if len(class_points) != len(points):
            print("warning: classified PCD point count differs; ignoring LAS classes", file=sys.stderr)
            classes = None

    bounds = pcd_bounds(points, margin=10.0)
    layers = make_orthophoto_layers(args.orthophoto)
    sam_layer = maybe_sam_tree_layer(args, layers["rgb"])
    if sam_layer is not None:
        layers["tree_mask"] = np.logical_or(layers["tree_mask"], sam_layer)
        layers["obstacle_distance_px"] = ndimage.distance_transform_edt(~layers["tree_mask"]).astype(np.float32)

    rows = compute_metrics(candidates, points, classes, layers, bounds, args)
    score_rows(rows, args.safest_threshold, args.safe_threshold)

    csv_out = prefix.with_suffix(".csv")
    html_out = prefix.with_suffix(".html")
    risk_png = prefix.with_name(prefix.name + "_orthophoto_risk.png")
    obstacle_png = prefix.with_name(prefix.name + "_obstacle_offset.png")
    tex_out = prefix.with_suffix(".tex")

    write_ranked_csv(rows, csv_out)
    risk_layer = 0.65 * layers["tree_score"] + 0.25 * layers["texture"] + 0.10 * (1.0 - layers["grass_score"])
    save_layer_png(risk_layer, risk_png)
    save_layer_png(-layers["obstacle_distance_px"], obstacle_png, cv2.COLORMAP_INFERNO)
    make_html(rows, points, args.orthophoto, bounds, html_out, args.html_max_points, args.orthophoto_stride)
    write_tex_report(rows, args, {"csv": csv_out, "html": html_out, "risk_png": risk_png, "obstacle_png": obstacle_png}, tex_out)

    counts = {cat: sum(1 for r in rows if r["risk_category"] == cat) for cat in ("safest", "safe", "risky")}
    print(f"Ranked {len(rows)} candidates")
    print(f"Safest: {counts['safest']}  Safe: {counts['safe']}  Risky: {counts['risky']}")
    print(f"Wrote ranked CSV: {csv_out}")
    print(f"Wrote HTML overlay: {html_out}")
    print(f"Wrote orthophoto risk layer: {risk_png}")
    print(f"Wrote obstacle offset layer: {obstacle_png}")
    print(f"Wrote TeX report: {tex_out}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(1)
