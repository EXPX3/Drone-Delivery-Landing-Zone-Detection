#!/usr/bin/env python3
"""Ground-truth-centric camera-only and camera-LiDAR localization comparison."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

from render_camera_lidar_audit import (COLORS, PCDS, ellipse, font, load_snapshot,
                                       pcd_bounds)


ROTATION = np.array([[0.94181787, -0.33612363], [0.33612363, 0.94181787]])
SURVEY_ORIGIN = np.array([696493.72806823, 5328131.61341409])
LOCAL_GT = {
    "Cargo 1": np.array([8.70, 8.50]),   # middle
    "Cargo 2": np.array([17.27, 8.50]),  # far right
    "Cargo 3": np.array([2.80, 8.50]),   # left
    "Person 1": np.array([8.70, 2.88]),
}
GT = {name: ROTATION @ value + SURVEY_ORIGIN for name, value in LOCAL_GT.items()}


def weighted_camera_group(items: list[dict], truth: np.ndarray) -> dict:
    weights = np.asarray([item["observations"] for item in items], dtype=float)
    centers = np.asarray([item["center"] for item in items], dtype=float)
    covariances = np.asarray([item["covariance"] for item in items], dtype=float)
    total = float(weights.sum())
    center = np.sum(weights[:, None] * centers, axis=0) / total
    offsets = centers - center
    covariance = np.sum(weights[:, None, None] * (
        covariances + offsets[:, :, None] * offsets[:, None, :]
    ), axis=0) / total
    errors = np.linalg.norm(centers - truth, axis=1)
    return {
        "center": center.tolist(), "covariance": covariance.tolist(),
        "proposal_modes": len(items), "observations": int(weights.sum()),
        "center_error_m": float(np.linalg.norm(center - truth)),
        "mode_error_weighted_rmse_m": float(np.sqrt(np.sum(weights * errors ** 2) / total)),
        "mode_error_maximum_m": float(errors.max()),
        "proposal_ids": [item["id"] for item in items],
        "mode_errors_m": errors.tolist(),
    }


def validated_objects(path: Path) -> dict[str, dict]:
    report = json.loads(path.read_text(encoding="utf-8"))
    tracks = {item["track_id"]: item for item in report["guided_lidar_tracks"]
              if item["confirmed"]}
    mapping = {"Cargo 1": 1, "Cargo 2": 2, "Cargo 3": 4, "Person 1": 5}
    output = {}
    for name, track_id in mapping.items():
        item = tracks[track_id]
        truth = GT[name]
        center = np.asarray(item["centroid"][:2], dtype=float)
        output[name] = {
            "center": center.tolist(),
            "covariance": item["empirical_covariance_xy_m2"],
            "observations": item["observations"],
            "radial_p95_m": item["empirical_radial_p95_m"],
            "ground_truth_error_m": float(np.linalg.norm(center - truth)),
        }
    return output


def build_results(camera: list[dict], live_lidar: list[dict], validated: Path) -> dict:
    groups = {name: [] for name in GT}
    vehicle_camera = []
    for item in camera:
        if item["class_name"] == "cargo_box":
            names = ["Cargo 1", "Cargo 2", "Cargo 3"]
        elif item["class_name"] == "person":
            names = ["Person 1"]
        elif item["class_name"] == "vehicle":
            vehicle_camera.append(item)
            continue
        else:
            continue
        nearest = min(names, key=lambda name: np.linalg.norm(
            np.asarray(item["center"]) - GT[name]))
        groups[nearest].append(item)

    lidar = validated_objects(validated)
    objects = {}
    for name, items in groups.items():
        objects[name] = {
            "ground_truth_local_xy_m": LOCAL_GT[name].tolist(),
            "ground_truth_utm_en_m": GT[name].tolist(),
            "camera_only": weighted_camera_group(items, GT[name]),
            "camera_correlated_lidar": lidar[name],
            "camera_modes": items,
        }

    live_vehicle = [item for item in live_lidar if item["class_name"] == "vehicle"]
    if vehicle_camera and live_vehicle:
        reference = max(live_vehicle, key=lambda item: item.get("p95_m", 0.0))
        center = np.asarray(reference["center"])
        aggregate = weighted_camera_group(vehicle_camera, center)
        objects["Vehicle 1"] = {
            "ground_truth_local_xy_m": None, "ground_truth_utm_en_m": None,
            "camera_only": aggregate,
            "camera_correlated_lidar": reference,
            "camera_modes": vehicle_camera,
            "note": "No surveyed vehicle ground truth; LiDAR track is the reference.",
        }
    return {"assignment": "nearest same-class surveyed center", "objects": objects}


def dashed(draw, first, second, fill, width=3, dash=10, gap=7):
    a, b = np.asarray(first), np.asarray(second)
    delta = b - a
    length = float(np.linalg.norm(delta))
    if length == 0:
        return
    unit = delta / length
    for start in np.arange(0, length, dash + gap):
        stop = min(start + dash, length)
        draw.line((*tuple(a + unit * start), *tuple(a + unit * stop)), fill=fill, width=width)


def render(results: dict, orthophoto: Path, pcd: Path, output: Path):
    objects = results["objects"]
    points = []
    for item in objects.values():
        if item["ground_truth_utm_en_m"] is not None:
            points.append(item["ground_truth_utm_en_m"])
        points.append(item["camera_only"]["center"])
        points.append(item["camera_correlated_lidar"]["center"])
        points.extend(mode["center"] for mode in item["camera_modes"])
    points = np.asarray(points)
    margin = 7.0
    bounds = (points[:, 0].min() - margin, points[:, 1].min() - margin,
              points[:, 0].max() + margin, points[:, 1].max() + margin)
    source = Image.open(orthophoto).convert("RGB")
    sb = pcd_bounds(pcd)

    def source_pixel(point):
        x, y = point
        return ((x - sb[0]) / (sb[2] - sb[0]) * (source.width - 1),
                (sb[3] - y) / (sb[3] - sb[1]) * (source.height - 1))

    left, bottom = source_pixel((bounds[0], bounds[1]))
    right, top = source_pixel((bounds[2], bounds[3]))
    map_size, panel = 1300, 650
    crop = source.crop((int(left), int(top), int(right) + 1, int(bottom) + 1))
    crop = crop.resize((map_size, map_size), Image.Resampling.LANCZOS)
    canvas = Image.new("RGB", (map_size + panel, map_size), (246, 248, 250))
    canvas.paste(crop, (0, 0))
    draw = ImageDraw.Draw(canvas, "RGBA")

    def pixel(point):
        x, y = point
        return ((x - bounds[0]) / (bounds[2] - bounds[0]) * map_size,
                (bounds[3] - y) / (bounds[3] - bounds[1]) * map_size)

    # Survey axes and scale follow the earlier ground-truth figure.
    origin_px = pixel(SURVEY_ORIGIN)
    x_end = pixel(SURVEY_ORIGIN + ROTATION @ [22.0, 0.0])
    y_end = pixel(SURVEY_ORIGIN + ROTATION @ [0.0, 12.0])
    draw.line((*origin_px, *x_end), fill=(235, 45, 55, 245), width=7)
    draw.line((*origin_px, *y_end), fill=(30, 190, 75, 245), width=7)
    draw.text(x_end, "+X [m]", font=font(18, True), fill=(235, 45, 55), anchor="lm")
    draw.text(y_end, "+Y [m]", font=font(18, True), fill=(30, 190, 75), anchor="mb")
    north = (55, 100)
    draw.line((north[0], north[1], north[0], 25), fill="white", width=8)
    draw.polygon(((north[0], 12), (north[0] - 12, 35), (north[0] + 12, 35)), fill="white")
    draw.text((north[0], 108), "N", font=font(20, True), fill="white", anchor="ma")
    scale_px = abs(pixel((bounds[0] + 5, bounds[1]))[0] - pixel((bounds[0], bounds[1]))[0])
    draw.line((55, 1245, 55 + scale_px, 1245), fill="white", width=8)
    draw.text((55 + scale_px / 2, 1232), "5 m", font=font(18, True), fill="white",
              stroke_width=2, stroke_fill=(30, 30, 30), anchor="mb")

    for name, item in objects.items():
        class_name = ("cargo_box" if name.startswith("Cargo") else
                      "person" if name.startswith("Person") else "vehicle")
        color = COLORS[class_name]
        truth = item["ground_truth_utm_en_m"]
        camera = item["camera_only"]
        lidar = item["camera_correlated_lidar"]

        # Raw proposal modes show why the aggregate camera spread is broad.
        for mode in item["camera_modes"]:
            x, y = pixel(mode["center"])
            draw.ellipse((x - 6, y - 6, x + 6, y + 6), fill=(15, 15, 15, 180),
                         outline=(*color, 230), width=3)
            if truth is not None:
                dashed(draw, (x, y), pixel(truth), (*color, 100), width=2)

        camera_polygon = [pixel(value) for value in ellipse(
            np.asarray(camera["center"]), np.asarray(camera["covariance"])
        )]
        dashed(draw, camera_polygon[0], camera_polygon[-1], (*color, 230), width=3)
        for first, second in zip(camera_polygon[:-1:3], camera_polygon[1::3]):
            draw.line((*first, *second), fill=(*color, 230), width=4)
        cx, cy = pixel(camera["center"])
        draw.ellipse((cx - 11, cy - 11, cx + 11, cy + 11), fill=(20, 20, 20, 210),
                     outline=(*color, 255), width=5)

        lidar_polygon = [pixel(value) for value in ellipse(
            np.asarray(lidar["center"]), np.asarray(lidar["covariance"])
        )]
        draw.polygon(lidar_polygon, fill=(55, 58, 64, 65))
        draw.line(lidar_polygon, fill=(48, 52, 58, 245), width=6, joint="curve")
        lx, ly = pixel(lidar["center"])
        draw.rectangle((lx - 9, ly - 9, lx + 9, ly + 9), fill=(*color, 255),
                       outline=(10, 10, 10), width=3)

        if truth is not None:
            tx, ty = pixel(truth)
            draw.line((tx - 11, ty, tx + 11, ty), fill=(0, 235, 255), width=5)
            draw.line((tx, ty - 11, tx, ty + 11), fill=(0, 235, 255), width=5)
            dashed(draw, (cx, cy), (tx, ty), (*color, 220), width=4)
            draw.line((lx, ly, tx, ty), fill=(48, 52, 58, 240), width=4)
            anchor = (tx + 15, ty - 15)
        else:
            anchor = (lx + 15, ly - 15)
        draw.text(anchor, name, font=font(18, True), fill=color,
                  stroke_width=3, stroke_fill=(20, 20, 20))

    x0 = map_size + 28
    draw.text((x0, 25), "Ground-Truth Localization Comparison", font=font(26, True), fill=(20, 27, 35))
    draw.text((x0, 63), "Bayern DOP20 | EPSG:25832 | East/North metres", font=font(15), fill=(75, 85, 95))
    yy = 103
    draw.text((x0, yy), "Symbols", font=font(20, True), fill=(20, 27, 35)); yy += 35
    draw.ellipse((x0, yy, x0 + 18, yy + 18), fill=(20, 20, 20), outline=(255, 220, 0), width=4)
    draw.text((x0 + 30, yy - 2), "Camera-only aggregate (dashed 95% spread)", font=font(14), fill=(30, 35, 40)); yy += 30
    draw.rectangle((x0, yy, x0 + 18, yy + 18), fill=(255, 220, 0), outline=(10, 10, 10), width=2)
    draw.text((x0 + 30, yy - 2), "Camera-correlated LiDAR estimate", font=font(14), fill=(30, 35, 40)); yy += 30
    draw.line((x0, yy + 9, x0 + 18, yy + 9), fill=(0, 220, 245), width=5)
    draw.line((x0 + 9, yy, x0 + 9, yy + 18), fill=(0, 220, 245), width=5)
    draw.text((x0 + 30, yy - 2), "Survey ground-truth center", font=font(14), fill=(30, 35, 40)); yy += 44

    for name in ("Cargo 1", "Cargo 2", "Cargo 3", "Person 1", "Vehicle 1"):
        if name not in objects:
            continue
        item = objects[name]
        camera, lidar = item["camera_only"], item["camera_correlated_lidar"]
        draw.text((x0, yy), name, font=font(19, True), fill=(25, 30, 36)); yy += 27
        if item["ground_truth_local_xy_m"] is not None:
            local = item["ground_truth_local_xy_m"]
            draw.text((x0, yy), f"GT local XY: ({local[0]:.2f}, {local[1]:.2f}) m",
                      font=font(14), fill=(55, 63, 72)); yy += 22
            draw.text((x0, yy),
                      f"Camera-only: {camera['proposal_modes']} modes / {camera['observations']} observations",
                      font=font(14), fill=(55, 63, 72)); yy += 22
            draw.text((x0, yy),
                      f"  center error {camera['center_error_m']:.2f} m | mode RMSE {camera['mode_error_weighted_rmse_m']:.2f} m | max {camera['mode_error_maximum_m']:.2f} m",
                      font=font(13), fill=(55, 63, 72)); yy += 22
            draw.text((x0, yy),
                      f"Camera+LiDAR: GT error {lidar['ground_truth_error_m']:.2f} m | P95 {lidar['radial_p95_m']:.2f} m",
                      font=font(13), fill=(55, 63, 72)); yy += 22
        else:
            draw.text((x0, yy), "No surveyed ground truth; LiDAR used as reference.",
                      font=font(14), fill=(55, 63, 72)); yy += 22
            draw.text((x0, yy),
                      f"Camera-only: {camera['proposal_modes']} modes | reference RMSE {camera['mode_error_weighted_rmse_m']:.2f} m",
                      font=font(13), fill=(55, 63, 72)); yy += 22
        yy += 17

    draw.text((x0, 1235),
              "Cargo naming matches confirmed_objects_ground_truth_axes_v3.png.",
              font=font(13), fill=(75, 85, 95), anchor="ls")
    output.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(output, quality=95)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--markers", type=Path, required=True)
    parser.add_argument("--validated", type=Path, default=PCDS / "semantic_experiment/retained_tracks_validation.json")
    parser.add_argument("--orthophoto", type=Path, default=PCDS / "696_5328_cropped_by_bayernatlas_20260812084517_satellite_overlay_dop20.png")
    parser.add_argument("--pcd", type=Path, default=PCDS / "696_5328_cropped_by_bayernatlas_20260812084517.pcd")
    parser.add_argument("--output", type=Path, default=PCDS / "semantic_experiment/camera_lidar_ground_truth_comparison.png")
    parser.add_argument("--json", type=Path, default=PCDS / "semantic_experiment/camera_lidar_ground_truth_comparison.json")
    args = parser.parse_args()
    camera, live_lidar, frame = load_snapshot(args.markers, args.validated)
    results = build_results(camera, live_lidar, args.validated)
    results["frame"] = frame
    render(results, args.orthophoto, args.pcd, args.output)
    args.json.write_text(json.dumps(results, indent=2) + "\n", encoding="utf-8")
    print(args.output); print(args.json)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
