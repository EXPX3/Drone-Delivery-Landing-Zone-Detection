#!/usr/bin/env python3
"""Render confirmed semantic tracks and covariance metrics on the DOP20 image."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[2]
PCDS = ROOT / "pcds"


def read_pcd_bounds(path: Path, margin_m: float = 10.0) -> tuple[float, float, float, float]:
    with path.open(encoding="ascii") as handle:
        for line in handle:
            if line.strip().lower() == "data ascii":
                break
        points = np.loadtxt(handle, usecols=(0, 1))
    minimum = np.min(points, axis=0) - margin_m
    maximum = np.max(points, axis=0) + margin_m
    return float(minimum[0]), float(minimum[1]), float(maximum[0]), float(maximum[1])


def world_to_source(x: float, y: float, bounds: tuple[float, float, float, float],
                    size: tuple[int, int]) -> tuple[float, float]:
    min_x, min_y, max_x, max_y = bounds
    width, height = size
    return ((x - min_x) / (max_x - min_x) * (width - 1),
            (max_y - y) / (max_y - min_y) * (height - 1))


def font(size: int, bold: bool = False) -> ImageFont.FreeTypeFont:
    name = "DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf"
    return ImageFont.truetype(f"/usr/share/fonts/truetype/dejavu/{name}", size)


def covariance_ellipse(center: np.ndarray, covariance: np.ndarray,
                       scale: float = np.sqrt(5.991), samples: int = 96) -> np.ndarray:
    values, vectors = np.linalg.eigh(covariance)
    order = np.argsort(values)[::-1]
    axes = scale * np.sqrt(np.maximum(values[order], 0.0))
    angles = np.linspace(0.0, 2.0 * np.pi, samples)
    offsets = (vectors[:, order] @ np.vstack((axes[0] * np.cos(angles),
                                               axes[1] * np.sin(angles)))).T
    return center[None, :] + offsets


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--metrics", type=Path,
                        default=PCDS / "semantic_experiment/retained_tracks_validation.json")
    parser.add_argument("--orthophoto", type=Path, default=PCDS /
                        "696_5328_cropped_by_bayernatlas_20260812084517_satellite_overlay_dop20.png")
    parser.add_argument("--pcd", type=Path, default=PCDS /
                        "696_5328_cropped_by_bayernatlas_20260812084517.pcd")
    parser.add_argument("--output", type=Path, default=PCDS / "semantic_experiment" /
                        "confirmed_objects_orthophoto_metrics.png")
    parser.add_argument("--context-m", type=float, default=12.0)
    args = parser.parse_args()

    report = json.loads(args.metrics.read_text(encoding="utf-8"))
    tracks = [item for item in report["guided_lidar_tracks"] if item["confirmed"]]
    cargo = sorted((item for item in tracks if item["class_name"] == "cargo_box"),
                   key=lambda item: item["track_id"])
    people = sorted((item for item in tracks if item["class_name"] == "person"),
                    key=lambda item: item["track_id"])
    selected = cargo + people
    if not selected:
        raise RuntimeError("No confirmed semantic tracks found")
    for index, item in enumerate(cargo, 1):
        item["display_name"] = f"Cargo {index}"
    for index, item in enumerate(people, 1):
        item["display_name"] = f"Person {index}"

    image = Image.open(args.orthophoto).convert("RGB")
    source_bounds = read_pcd_bounds(args.pcd)
    centers = np.asarray([item["centroid"][:2] for item in selected], dtype=np.float64)
    crop_bounds = (float(np.min(centers[:, 0]) - args.context_m),
                   float(np.min(centers[:, 1]) - args.context_m),
                   float(np.max(centers[:, 0]) + args.context_m),
                   float(np.max(centers[:, 1]) + args.context_m))
    left, bottom = world_to_source(crop_bounds[0], crop_bounds[1], source_bounds, image.size)
    right, top = world_to_source(crop_bounds[2], crop_bounds[3], source_bounds, image.size)
    crop = image.crop((int(left), int(top), int(right) + 1, int(bottom) + 1))
    map_size = 1200
    crop = crop.resize((map_size, map_size), Image.Resampling.LANCZOS)
    canvas = Image.new("RGB", (1900, 1200), (247, 248, 250))
    canvas.paste(crop, (0, 0))
    draw = ImageDraw.Draw(canvas, "RGBA")

    def map_pixel(x: float, y: float) -> tuple[float, float]:
        min_x, min_y, max_x, max_y = crop_bounds
        return ((x - min_x) / (max_x - min_x) * map_size,
                (max_y - y) / (max_y - min_y) * map_size)

    colors = {"cargo_box": (255, 225, 0, 255), "person": (255, 0, 220, 255)}
    # Relative geometry is shown first so object symbols remain unobscured.
    pairs = []
    for first_index in range(len(selected)):
        for second_index in range(first_index + 1, len(selected)):
            first, second = selected[first_index], selected[second_index]
            a = np.asarray(first["centroid"][:2])
            b = np.asarray(second["centroid"][:2])
            distance = float(np.linalg.norm(b - a))
            pairs.append((first["display_name"], second["display_name"], b - a, distance))
            p0, p1 = map_pixel(*a), map_pixel(*b)
            draw.line((p0, p1), fill=(255, 255, 255, 165), width=3)
            middle = ((p0[0] + p1[0]) * 0.5, (p0[1] + p1[1]) * 0.5)
            draw.rounded_rectangle((middle[0] - 38, middle[1] - 15,
                                    middle[0] + 38, middle[1] + 15), radius=4,
                                   fill=(10, 15, 20, 190))
            draw.text(middle, f"{distance:.2f} m", font=font(18, True),
                      fill="white", anchor="mm")

    for item in selected:
        center = np.asarray(item["centroid"][:2], dtype=np.float64)
        covariance = np.asarray(item["empirical_covariance_xy_m2"], dtype=np.float64)
        ellipse = covariance_ellipse(center, covariance)
        polygon = [map_pixel(float(point[0]), float(point[1])) for point in ellipse]
        color = colors[item["class_name"]]
        draw.polygon(polygon, fill=(*color[:3], 55))
        draw.line(polygon, fill=color, width=7, joint="curve")
        x, y = map_pixel(*center)
        radius = 13
        draw.ellipse((x - radius, y - radius, x + radius, y + radius),
                     fill=color, outline=(10, 10, 10, 255), width=3)
        draw.rounded_rectangle((x + 16, y - 29, x + 142, y + 4), radius=5,
                               fill=(10, 15, 20, 220))
        draw.text((x + 24, y - 25), item["display_name"], font=font(20, True),
                  fill=color)

    # North arrow and metric scale bar.
    draw.line((55, 105, 55, 35), fill="white", width=7)
    draw.polygon(((55, 22), (43, 45), (67, 45)), fill="white")
    draw.text((55, 112), "N", font=font(22, True), fill="white", anchor="ma")
    meters_per_pixel = (crop_bounds[2] - crop_bounds[0]) / map_size
    bar_px = 5.0 / meters_per_pixel
    draw.line((55, 1150, 55 + bar_px, 1150), fill="white", width=8)
    draw.text((55 + bar_px / 2, 1120), "5 m", font=font(20, True), fill="white", anchor="mm")

    panel_x = 1230
    draw.text((panel_x, 30), "Confirmed semantic objects", font=font(30, True),
              fill=(20, 27, 38, 255))
    draw.text((panel_x, 72), "Bayern DOP20 | UTM 32N | 95% XY covariance",
              font=font(17), fill=(60, 70, 85, 255))
    y = 118
    for item in selected:
        covariance = np.asarray(item["empirical_covariance_xy_m2"])
        center = item["centroid"]
        color = colors[item["class_name"]]
        draw.ellipse((panel_x, y + 3, panel_x + 18, y + 21), fill=color)
        draw.text((panel_x + 30, y), item["display_name"], font=font(21, True),
                  fill=(20, 27, 38, 255))
        y += 31
        lines = [
            f"E {center[0]:.3f} m   N {center[1]:.3f} m   Z {center[2]:.3f} m",
            f"observations {item['observations']}   radial P95 {item['empirical_radial_p95_m']:.3f} m",
            f"Cov XY [m^2]  [{covariance[0, 0]:.4f}  {covariance[0, 1]:+.4f}]",
            f"              [{covariance[1, 0]:+.4f}  {covariance[1, 1]:.4f}]",
        ]
        for line in lines:
            draw.text((panel_x + 6, y), line, font=font(15), fill=(45, 52, 64, 255))
            y += 23
        y += 14

    draw.line((panel_x, y, 1870, y), fill=(180, 185, 195, 255), width=2)
    y += 15
    draw.text((panel_x, y), "Relative positions", font=font(21, True), fill=(20, 27, 38, 255))
    y += 33
    for first, second, delta, distance in pairs:
        if first.startswith("Person") or second.startswith("Person") or (
                first.startswith("Cargo") and second.startswith("Cargo")):
            draw.text((panel_x, y),
                      f"{first} -> {second}: d={distance:.2f} m, dE={delta[0]:+.2f}, dN={delta[1]:+.2f}",
                      font=font(14), fill=(45, 52, 64, 255))
            y += 22

    draw.text((panel_x, 1160),
              "Ellipse: chi-square 95% contour (2 DoF). Metrics are empirical across observations.",
              font=font(13), fill=(85, 92, 104, 255))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(args.output, quality=95)
    print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
