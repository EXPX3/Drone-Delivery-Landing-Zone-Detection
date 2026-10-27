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

# Canonical report names mapped to the unlabeled local survey coordinates. The
# rigid transform is the full-bag fit documented in SEMANTIC_OBJECT_LOCALIZATION_RESULTS.md.
GROUND_TRUTH_LOCAL_XY = {
    "Cargo 1": np.array([8.70, 8.50]),
    "Cargo 2": np.array([17.27, 8.50]),
    "Cargo 3": np.array([2.80, 8.50]),
    "Person 1": np.array([8.70, 2.88]),
}
GROUND_TRUTH_ROTATION = np.array([
    [0.94181787, -0.33612363],
    [0.33612363, 0.94181787],
])
GROUND_TRUTH_ORIGIN_UTM = np.array([696493.72806823, 5328131.61341409])
UAV_START_UTM = np.array([696496.793439017, 5328128.724636963])


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


def dashed_line(draw: ImageDraw.ImageDraw, start: tuple[float, float],
                end: tuple[float, float], fill: tuple[int, ...], width: int = 3,
                dash_px: float = 12.0, gap_px: float = 8.0) -> None:
    """Draw a metric-independent dashed screen-space line."""
    first = np.asarray(start, dtype=np.float64)
    delta = np.asarray(end, dtype=np.float64) - first
    length = float(np.linalg.norm(delta))
    if length < 1e-9:
        return
    direction = delta / length
    position = 0.0
    while position < length:
        stop = min(position + dash_px, length)
        segment_start = first + direction * position
        segment_end = first + direction * stop
        draw.line((*segment_start, *segment_end), fill=fill, width=width)
        position += dash_px + gap_px


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
    parser.add_argument("--ground-truth-axes-layout", action="store_true",
                        help="Render the uncluttered survey-axis comparison variant")
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
    ground_truth = {
        name: GROUND_TRUTH_ROTATION @ local + GROUND_TRUTH_ORIGIN_UTM
        for name, local in GROUND_TRUTH_LOCAL_XY.items()
    }

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
            if not args.ground_truth_axes_layout:
                draw.rounded_rectangle((middle[0] - 38, middle[1] - 15,
                                        middle[0] + 38, middle[1] + 15), radius=4,
                                       fill=(10, 15, 20, 190))
                draw.text(middle, f"{distance:.2f} m", font=font(18, True),
                          fill="white", anchor="mm")

    if args.ground_truth_axes_layout:
        # ROS convention: +X red and +Y green. Projection lines encode each
        # object's local x/y values without assigning any ground-truth Z.
        origin = GROUND_TRUTH_ORIGIN_UTM
        x_end = GROUND_TRUTH_ROTATION @ np.array([20.0, 0.0]) + origin
        y_end = GROUND_TRUTH_ROTATION @ np.array([0.0, 12.0]) + origin
        origin_px = map_pixel(*origin)
        x_end_px = map_pixel(*x_end)
        y_end_px = map_pixel(*y_end)
        x_screen_unit = ((np.asarray(x_end_px) - np.asarray(origin_px)) /
                         np.linalg.norm(np.asarray(x_end_px) - np.asarray(origin_px)))
        y_screen_unit = ((np.asarray(y_end_px) - np.asarray(origin_px)) /
                         np.linalg.norm(np.asarray(y_end_px) - np.asarray(origin_px)))
        draw.line((*origin_px, *x_end_px), fill=(235, 45, 55, 245), width=7)
        draw.line((*origin_px, *y_end_px), fill=(35, 190, 80, 245), width=7)
        for endpoint, color in ((x_end_px, (235, 45, 55, 255)),
                                (y_end_px, (35, 190, 80, 255))):
            vector = np.asarray(endpoint) - np.asarray(origin_px)
            unit = vector / np.linalg.norm(vector)
            normal = np.array([-unit[1], unit[0]])
            tip = np.asarray(endpoint)
            back = tip - unit * 20.0
            draw.polygon((tuple(tip), tuple(back + normal * 9.0),
                          tuple(back - normal * 9.0)), fill=color)
        draw.text((x_end_px[0] + 8, x_end_px[1]), "+X", font=font(19, True),
                  fill=(235, 45, 55, 255), anchor="lm")
        draw.text((y_end_px[0], y_end_px[1] - 8), "+Y", font=font(19, True),
                  fill=(35, 190, 80, 255), anchor="mb")
        labelled_x = set()
        labelled_y = set()
        for name, truth in ground_truth.items():
            local = GROUND_TRUTH_LOCAL_XY[name]
            on_x = GROUND_TRUTH_ROTATION @ np.array([local[0], 0.0]) + origin
            on_y = GROUND_TRUTH_ROTATION @ np.array([0.0, local[1]]) + origin
            truth_px = map_pixel(*truth)
            on_x_px = map_pixel(*on_x)
            on_y_px = map_pixel(*on_y)

            # Standard Cartesian orthographic projection onto both axes. Axis
            # colors follow ROS REP-103: X red and Y green.
            dashed_line(draw, truth_px, on_y_px, (235, 45, 55, 190), width=3)
            dashed_line(draw, on_x_px, truth_px, (35, 190, 80, 205), width=3)
            x_key = round(float(local[0]), 3)
            if x_key not in labelled_x:
                tick_a = np.asarray(on_x_px) - y_screen_unit * 7.0
                tick_b = np.asarray(on_x_px) + y_screen_unit * 7.0
                draw.line((*tick_a, *tick_b), fill=(235, 45, 55, 255), width=3)
                label_position = np.asarray(on_x_px) - y_screen_unit * 20.0
                draw.text(tuple(label_position), f"{local[0]:.2f} m",
                          font=font(13, True),
                          fill=(235, 45, 55, 255), stroke_width=3,
                          stroke_fill=(247, 248, 250, 225), anchor="mm")
                labelled_x.add(x_key)
            y_key = round(float(local[1]), 3)
            if y_key not in labelled_y:
                tick_a = np.asarray(on_y_px) - x_screen_unit * 7.0
                tick_b = np.asarray(on_y_px) + x_screen_unit * 7.0
                draw.line((*tick_a, *tick_b), fill=(35, 190, 80, 255), width=3)
                label_position = np.asarray(on_y_px) - x_screen_unit * 27.0
                draw.text(tuple(label_position), f"{local[1]:.2f} m",
                          font=font(13, True), fill=(25, 155, 65, 255),
                          stroke_width=3, stroke_fill=(247, 248, 250, 225),
                          anchor="mm")
                labelled_y.add(y_key)

    # Ground-truth residual arrows are drawn below object symbols and covariance.
    residuals = {}
    for item in selected:
        name = item["display_name"]
        detected = np.asarray(item["centroid"][:2], dtype=np.float64)
        truth = ground_truth[name]
        residuals[name] = detected - truth
        p_truth, p_detected = map_pixel(*truth), map_pixel(*detected)
        draw.line((p_truth, p_detected), fill=(0, 235, 255, 255), width=5)

    for item in selected:
        center = np.asarray(item["centroid"][:2], dtype=np.float64)
        covariance = np.asarray(item["empirical_covariance_xy_m2"], dtype=np.float64)
        ellipse = covariance_ellipse(center, covariance)
        polygon = [map_pixel(float(point[0]), float(point[1])) for point in ellipse]
        color = colors[item["class_name"]]
        ellipse_color = ((48, 52, 58, 220) if args.ground_truth_axes_layout else color)
        ellipse_fill = ((55, 59, 65, 70) if args.ground_truth_axes_layout else (*color[:3], 55))
        draw.polygon(polygon, fill=ellipse_fill)
        draw.line(polygon, fill=ellipse_color, width=7, joint="curve")
        x, y = map_pixel(*center)
        radius = 7 if args.ground_truth_axes_layout else 13
        draw.ellipse((x - radius, y - radius, x + radius, y + radius),
                     fill=color, outline=(10, 10, 10, 255), width=3)
        draw.rounded_rectangle((x + 16, y - 29, x + 142, y + 4), radius=5,
                               fill=(10, 15, 20, 220))
        draw.text((x + 24, y - 25), item["display_name"], font=font(20, True),
                  fill=color)

    # Cyan crosses are ground truth; the white square and triangle distinguish
    # the inferred survey origin and independently recovered UAV start.
    for name, truth in ground_truth.items():
        x, y = map_pixel(*truth)
        size = 11
        draw.line((x - size, y, x + size, y), fill=(0, 235, 255, 255), width=5)
        draw.line((x, y - size, x, y + size), fill=(0, 235, 255, 255), width=5)
    origin_px = map_pixel(*GROUND_TRUTH_ORIGIN_UTM)
    draw.rectangle((origin_px[0] - 9, origin_px[1] - 9,
                    origin_px[0] + 9, origin_px[1] + 9),
                   fill=(255, 255, 255, 255), outline=(10, 15, 20, 255), width=3)
    start_px = map_pixel(*UAV_START_UTM)
    draw.polygon(((start_px[0], start_px[1] - 13),
                  (start_px[0] - 12, start_px[1] + 10),
                  (start_px[0] + 12, start_px[1] + 10)),
                 fill=(80, 190, 255, 255), outline=(10, 15, 20, 255))

    # North arrow and metric scale bar.
    draw.line((55, 105, 55, 35), fill="white", width=7)
    draw.polygon(((55, 22), (43, 45), (67, 45)), fill="white")
    draw.text((55, 112), "N", font=font(22, True), fill="white", anchor="ma")
    meters_per_pixel = (crop_bounds[2] - crop_bounds[0]) / map_size
    bar_px = 5.0 / meters_per_pixel
    draw.line((55, 1150, 55 + bar_px, 1150), fill="white", width=8)
    draw.text((55 + bar_px / 2, 1120), "5 m", font=font(20, True), fill="white", anchor="mm")

    panel_x = 1230
    title = ("Detection vs ground-truth layout" if args.ground_truth_axes_layout
             else "Confirmed semantic objects")
    draw.text((panel_x, 30), title, font=font(30, True),
              fill=(20, 27, 38, 255))
    draw.text((panel_x, 72), "Bayern DOP20 | EPSG:25832 | 95% East/North covariance",
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
            f"Cov EN [m^2]  [{covariance[0, 0]:.4f}  {covariance[0, 1]:+.4f}]",
            f"              [{covariance[1, 0]:+.4f}  {covariance[1, 1]:.4f}]",
        ]
        for line in lines:
            draw.text((panel_x + 6, y), line, font=font(15), fill=(45, 52, 64, 255))
            y += 23
        y += 14

    if not args.ground_truth_axes_layout:
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

    y += 8
    draw.line((panel_x, y, 1870, y), fill=(180, 185, 195, 255), width=2)
    y += 14
    draw.text((panel_x, y), "Provided ground truth", font=font(21, True),
              fill=(20, 27, 38, 255))
    y += 31
    errors = []
    for item in selected:
        name = item["display_name"]
        local = GROUND_TRUTH_LOCAL_XY[name]
        residual = residuals[name]
        local_residual = GROUND_TRUTH_ROTATION.T @ residual
        error = float(np.linalg.norm(residual))
        errors.append(error)
        draw.text((panel_x, y),
                  f"{name}: local ({local[0]:.2f}, {local[1]:.2f}) m | error {error:.3f} m",
                  font=font(14, True), fill=(35, 48, 60, 255))
        y += 20
        draw.text((panel_x + 12, y),
                  f"error x={local_residual[0]:+.3f} m, error y={local_residual[1]:+.3f} m",
                  font=font(13), fill=(55, 65, 78, 255))
        y += 21
    rmse = float(np.sqrt(np.mean(np.square(errors))))
    y += 3
    draw.text((panel_x, y), f"Retained-center RMSE to full-bag GT fit: {rmse:.3f} m",
              font=font(14, True), fill=(20, 27, 38, 255))
    y += 23
    draw.text((panel_x, y),
              "white square: inferred survey origin | blue triangle: UAV start",
              font=font(13), fill=(55, 65, 78, 255))
    y += 20
    draw.text((panel_x, y),
              f"origin-to-start distance: {np.linalg.norm(UAV_START_UTM - GROUND_TRUTH_ORIGIN_UTM):.3f} m",
              font=font(13, True), fill=(55, 65, 78, 255))

    if args.ground_truth_axes_layout:
        y += 28
        draw.text((panel_x, y), "Coordinate construction", font=font(18, True),
                  fill=(20, 27, 38, 255))
        y += 27
        draw.text((panel_x, y), "red dashed: projection onto local Y-axis",
                  font=font(13), fill=(200, 38, 48, 255))
        y += 20
        draw.text((panel_x, y), "green dashed: projection onto local X-axis",
                  font=font(13), fill=(25, 150, 65, 255))
        y += 20
        draw.text((panel_x, y), "dark grey: empirical 95% covariance contour",
                  font=font(13), fill=(55, 59, 65, 255))

    draw.text((panel_x, 1160),
              "Solid: detection | cyan cross: ground truth | ellipse: empirical 95% covariance.",
              font=font(13), fill=(85, 92, 104, 255))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(args.output, quality=95)
    print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
