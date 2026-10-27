#!/usr/bin/env python3
"""Render camera-only and camera-correlated LiDAR localization on the DOP20."""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path

import numpy as np
import yaml
from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[2]
PCDS = ROOT / "pcds"
ORIGIN = np.array([696423.686, 5328133.500])
COLORS = {
    "cargo_box": (255, 220, 0),
    "person": (255, 0, 220),
    "vehicle": (0, 210, 255),
}


def font(size: int, bold: bool = False):
    name = "DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf"
    return ImageFont.truetype(f"/usr/share/fonts/truetype/dejavu/{name}", size)


def pcd_bounds(path: Path, margin: float = 10.0) -> tuple[float, float, float, float]:
    with path.open(encoding="ascii") as handle:
        for line in handle:
            if line.strip().lower() == "data ascii":
                break
        points = np.loadtxt(handle, usecols=(0, 1))
    low, high = points.min(axis=0) - margin, points.max(axis=0) + margin
    return float(low[0]), float(low[1]), float(high[0]), float(high[1])


def covariance_from_ellipse(marker: dict) -> np.ndarray:
    points = np.asarray([[p["x"], p["y"]] for p in marker["points"]], dtype=float)
    # RViz contour is a 2-sigma ellipse; uniform contour covariance is 2*Sigma.
    return np.cov(points, rowvar=False) / 2.0


def ellipse(center: np.ndarray, covariance: np.ndarray, scale=np.sqrt(5.991)) -> np.ndarray:
    values, vectors = np.linalg.eigh(0.5 * (covariance + covariance.T))
    order = np.argsort(values)[::-1]
    axes = scale * np.sqrt(np.maximum(values[order], 0.0))
    angles = np.linspace(0, 2 * np.pi, 97)
    return center + (vectors[:, order] @ np.vstack(
        (axes[0] * np.cos(angles), axes[1] * np.sin(angles))
    )).T


def load_snapshot(path: Path, validated: Path) -> tuple[list[dict], list[dict], str]:
    message = next(yaml.safe_load_all(path.read_text(encoding="utf-8")))
    markers = message["markers"]
    frame = next((m.get("text", "") for m in markers if m.get("ns") == "replay_sync"), "")
    labels = [m for m in markers if m.get("ns") == "vision_only_labels"]
    contours = [m for m in markers if m.get("ns") == "vision_only_covariance"]
    camera = []
    pattern = re.compile(
        r"(?P<class>\w+) \| vision provisional #(?P<id>\d+) \| conf (?P<confidence>[\d.]+)"
        r" \| (?:confirmed|tentative) \| obs (?P<observations>\d+) \| last seen (?P<age>[\d.]+)s"
    )
    for label, contour in zip(labels, contours):
        match = pattern.search(label["text"])
        if not match:
            continue
        position = label["pose"]["position"]
        camera.append({
            "kind": "camera", "class_name": match.group("class"),
            "id": int(match.group("id")),
            "center": (ORIGIN + [position["x"], position["y"]]).tolist(),
            "covariance": covariance_from_ellipse(contour).tolist(),
            "confidence": float(match.group("confidence")),
            "observations": int(match.group("observations")),
            "last_seen_s": float(match.group("age")),
        })

    lidar = []
    lidar_labels = [m for m in markers if m.get("ns") == "semantic_track_labels"]
    lidar_pattern = re.compile(
        r"(?P<class>\w+) #(?P<id>\d+) \| confirmed \| P95 (?P<p95>[\d.]+) m\n"
        r"Cov XY \[m\^2\] \[(?P<a>[\d.]+) (?P<b>[+-][\d.]+); [+-][\d.]+ (?P<d>[\d.]+)\]"
    )
    for label in lidar_labels:
        match = lidar_pattern.search(label["text"])
        if not match:
            continue
        position = label["pose"]["position"]
        lidar.append({
            "kind": "lidar", "class_name": match.group("class"),
            "id": int(match.group("id")),
            "center": (ORIGIN + [position["x"], position["y"]]).tolist(),
            "covariance": [[float(match.group("a")), float(match.group("b"))],
                           [float(match.group("b")), float(match.group("d"))]],
            "p95_m": float(match.group("p95")), "source": "live final state",
        })

    # The dynamic person is absent from the final live marker frame. Include its
    # validated full-bag camera-correlated LiDAR result so every requested class
    # is represented, and label the source explicitly.
    report = json.loads(validated.read_text(encoding="utf-8"))
    if not any(item["class_name"] == "person" for item in lidar):
        people = [item for item in report["guided_lidar_tracks"]
                  if item["confirmed"] and item["class_name"] == "person"]
        if people:
            item = max(people, key=lambda value: value["observations"])
            lidar.append({
                "kind": "lidar", "class_name": "person", "id": item["track_id"],
                "center": item["centroid"][:2],
                "covariance": item["empirical_covariance_xy_m2"],
                "p95_m": item["empirical_radial_p95_m"],
                "source": "validated full-bag track",
            })
    return camera, lidar, frame


def summarize(camera: list[dict], lidar: list[dict]) -> dict:
    classes = sorted(set(item["class_name"] for item in camera + lidar))
    result = {"classes": {}, "correlated_core_errors_m": []}
    for name in classes:
        c = [item for item in camera if item["class_name"] == name]
        l = [item for item in lidar if item["class_name"] == name]
        distances = []
        assignments = []
        for item in c:
            if not l:
                continue
            values = [float(np.linalg.norm(np.asarray(item["center"])
                                           - np.asarray(target["center"]))) for target in l]
            index = int(np.argmin(values))
            distances.append(values[index])
            assignments.append({"camera_id": item["id"], "lidar_id": l[index]["id"],
                                "distance_m": values[index]})
        values = np.asarray(distances)
        core = sorted(assignments, key=lambda item: item["distance_m"])[:min(len(l), len(assignments))]
        result["correlated_core_errors_m"].extend(item["distance_m"] for item in core)
        result["classes"][name] = {
            "camera_tracks": len(c), "lidar_tracks": len(l),
            "excess_camera_tracks": max(0, len(c) - len(l)),
            "nearest_lidar_error_m": {
                "median": float(np.median(values)) if len(values) else None,
                "rmse": float(np.sqrt(np.mean(values ** 2))) if len(values) else None,
                "p95": float(np.percentile(values, 95)) if len(values) else None,
                "maximum": float(np.max(values)) if len(values) else None,
            },
            "core_pairs": core, "all_assignments": assignments,
        }
    core = np.asarray(result["correlated_core_errors_m"])
    result["correlated_core_summary_m"] = {
        "count": len(core), "median": float(np.median(core)),
        "rmse": float(np.sqrt(np.mean(core ** 2))), "maximum": float(np.max(core)),
    }
    return result


def render(camera: list[dict], lidar: list[dict], summary: dict, orthophoto: Path,
           pcd: Path, output: Path, frame: str) -> None:
    centers = np.asarray([item["center"] for item in camera + lidar])
    margin = 8.0
    bounds = (centers[:, 0].min() - margin, centers[:, 1].min() - margin,
              centers[:, 0].max() + margin, centers[:, 1].max() + margin)
    source = Image.open(orthophoto).convert("RGB")
    sb = pcd_bounds(pcd)

    def source_pixel(x, y):
        return ((x - sb[0]) / (sb[2] - sb[0]) * (source.width - 1),
                (sb[3] - y) / (sb[3] - sb[1]) * (source.height - 1))

    left, bottom = source_pixel(bounds[0], bounds[1])
    right, top = source_pixel(bounds[2], bounds[3])
    map_size, panel = 1300, 560
    crop = source.crop((int(left), int(top), int(right) + 1, int(bottom) + 1))
    crop = crop.resize((map_size, map_size), Image.Resampling.LANCZOS)
    canvas = Image.new("RGB", (map_size + panel, map_size), (245, 247, 249))
    canvas.paste(crop, (0, 0))
    draw = ImageDraw.Draw(canvas, "RGBA")

    def pixel(point):
        x, y = point
        return ((x - bounds[0]) / (bounds[2] - bounds[0]) * map_size,
                (bounds[3] - y) / (bounds[3] - bounds[1]) * map_size)

    # Metric engineering grid and E/N axes.
    for x in np.arange(np.ceil(bounds[0] / 5) * 5, bounds[2], 5):
        px = pixel((x, bounds[1]))[0]
        draw.line((px, 0, px, map_size), fill=(255, 255, 255, 75), width=1)
        draw.text((px + 3, map_size - 7), f"{x - bounds[0]:.0f}", font=font(13),
                  fill="white", stroke_width=2, stroke_fill=(20, 20, 20), anchor="ls")
    for y in np.arange(np.ceil(bounds[1] / 5) * 5, bounds[3], 5):
        py = pixel((bounds[0], y))[1]
        draw.line((0, py, map_size, py), fill=(255, 255, 255, 75), width=1)
        draw.text((5, py - 3), f"{y - bounds[1]:.0f}", font=font(13), fill="white",
                  stroke_width=2, stroke_fill=(20, 20, 20))
    origin = (50, map_size - 50)
    draw.line((*origin, origin[0] + 120, origin[1]), fill=(235, 45, 55, 255), width=6)
    draw.line((*origin, origin[0], origin[1] - 120), fill=(35, 190, 80, 255), width=6)
    draw.text((origin[0] + 125, origin[1]), "+E / X [m]", font=font(18, True),
              fill=(235, 45, 55), anchor="lm")
    draw.text((origin[0], origin[1] - 125), "+N / Y [m]", font=font(18, True),
              fill=(35, 190, 80), anchor="mb")
    bar_start = (map_size - 280, map_size - 45)
    bar_end = pixel((bounds[0] + 10, bounds[1]))[0] - pixel((bounds[0], bounds[1]))[0]
    draw.line((*bar_start, bar_start[0] + bar_end, bar_start[1]), fill="white", width=8)
    draw.text((bar_start[0] + bar_end / 2, bar_start[1] - 12), "10 m",
              font=font(18, True), fill="white", stroke_width=2,
              stroke_fill=(20, 20, 20), anchor="mb")

    # Nearest same-class correspondence lines expose projection spread.
    for item in camera:
        candidates = [target for target in lidar if target["class_name"] == item["class_name"]]
        if not candidates:
            continue
        target = min(candidates, key=lambda value: np.linalg.norm(
            np.asarray(item["center"]) - np.asarray(value["center"])))
        distance = np.linalg.norm(np.asarray(item["center"]) - np.asarray(target["center"]))
        draw.line((*pixel(item["center"]), *pixel(target["center"])),
                  fill=(255, 255, 255, 125), width=2)
        middle = (np.asarray(pixel(item["center"])) + np.asarray(pixel(target["center"]))) / 2
        draw.text(tuple(middle), f"{distance:.1f}m", font=font(12, True), fill="white",
                  stroke_width=2, stroke_fill=(15, 15, 15), anchor="mm")

    for item in camera:
        color = COLORS[item["class_name"]]
        polygon = [pixel(value) for value in ellipse(
            np.asarray(item["center"]), np.asarray(item["covariance"])
        )]
        draw.line(polygon, fill=(*color, 190), width=3, joint="curve")
        x, y = pixel(item["center"])
        draw.ellipse((x - 8, y - 8, x + 8, y + 8), fill=(20, 20, 20, 180),
                     outline=(*color, 255), width=4)
        draw.text((x + 10, y - 10), f"C{item['id']}", font=font(14, True), fill=color,
                  stroke_width=2, stroke_fill=(15, 15, 15))
    for item in lidar:
        color = COLORS[item["class_name"]]
        polygon = [pixel(value) for value in ellipse(
            np.asarray(item["center"]), np.asarray(item["covariance"])
        )]
        draw.polygon(polygon, fill=(*color, 50))
        draw.line(polygon, fill=(*color, 255), width=6, joint="curve")
        x, y = pixel(item["center"])
        draw.rectangle((x - 9, y - 9, x + 9, y + 9), fill=(*color, 255),
                       outline=(10, 10, 10, 255), width=3)
        draw.text((x + 12, y + 8), f"L{item['id']}", font=font(15, True), fill=color,
                  stroke_width=2, stroke_fill=(15, 15, 15))

    x0 = map_size + 28
    draw.text((x0, 28), "Camera–LiDAR Localization Audit", font=font(26, True), fill=(20, 27, 35))
    draw.text((x0, 66), frame or "Final observed state", font=font(14), fill=(75, 85, 95))
    draw.text((x0, 108), "Legend", font=font(20, True), fill=(20, 27, 35))
    draw.ellipse((x0, 145, x0 + 18, 163), fill=(20, 20, 20), outline=(255, 220, 0), width=4)
    draw.text((x0 + 30, 143), "Camera-only center + hollow 95% covariance", font=font(14), fill=(30, 35, 40))
    draw.rectangle((x0, 180, x0 + 18, 198), fill=(255, 220, 0), outline=(10, 10, 10), width=2)
    draw.text((x0 + 30, 178), "Camera-correlated LiDAR + solid covariance", font=font(14), fill=(30, 35, 40))
    yy = 228
    for name, color in COLORS.items():
        draw.rectangle((x0, yy, x0 + 18, yy + 18), fill=color)
        draw.text((x0 + 30, yy - 2), name.replace("_", " "), font=font(15, True), fill=(30, 35, 40))
        yy += 30
    yy += 20
    draw.text((x0, yy), "Observed Error Spread", font=font(20, True), fill=(20, 27, 35)); yy += 38
    for name in ("cargo_box", "person", "vehicle"):
        value = summary["classes"].get(name)
        if not value:
            continue
        metric = value["nearest_lidar_error_m"]
        lines = [
            f"{name.replace('_', ' ').title()}: C={value['camera_tracks']}, L={value['lidar_tracks']}",
            f"  nearest-L median {metric['median']:.2f} m",
            f"  RMSE {metric['rmse']:.2f} m, max {metric['maximum']:.2f} m",
        ]
        for line in lines:
            draw.text((x0, yy), line, font=font(15, line == lines[0]), fill=(35, 42, 50)); yy += 25
        yy += 10
    core = summary["correlated_core_summary_m"]
    draw.text((x0, yy), "Correlated Core", font=font(20, True), fill=(20, 27, 35)); yy += 36
    draw.text((x0, yy), f"{core['count']} class-matched object pairs", font=font(15), fill=(35, 42, 50)); yy += 25
    draw.text((x0, yy), f"Median {core['median']:.2f} m | RMSE {core['rmse']:.2f} m", font=font(15), fill=(35, 42, 50)); yy += 25
    draw.text((x0, yy), f"Maximum {core['maximum']:.2f} m", font=font(15), fill=(35, 42, 50)); yy += 48
    draw.text((x0, yy), "Interpretation", font=font(20, True), fill=(20, 27, 35)); yy += 38
    for line in ("Small core error coexists with displaced modes.",
                 "The spread is correlated and non-Gaussian.",
                 "Camera class evidence is useful; camera-only",
                 "world position is not safety-authoritative."):
        draw.text((x0, yy), line, font=font(15), fill=(35, 42, 50)); yy += 25
    output.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(output, quality=95)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--markers", type=Path, required=True)
    parser.add_argument("--validated", type=Path, default=PCDS / "semantic_experiment/retained_tracks_validation.json")
    parser.add_argument("--orthophoto", type=Path, default=PCDS / "696_5328_cropped_by_bayernatlas_20260812084517_satellite_overlay_dop20.png")
    parser.add_argument("--pcd", type=Path, default=PCDS / "696_5328_cropped_by_bayernatlas_20260812084517.pcd")
    parser.add_argument("--output", type=Path, default=PCDS / "semantic_experiment/camera_lidar_final_state_audit.png")
    parser.add_argument("--json", type=Path, default=PCDS / "semantic_experiment/camera_lidar_final_state_audit.json")
    args = parser.parse_args()
    camera, lidar, frame = load_snapshot(args.markers, args.validated)
    summary = summarize(camera, lidar)
    summary.update({"frame": frame, "camera_tracks": camera, "lidar_tracks": lidar})
    render(camera, lidar, summary, args.orthophoto, args.pcd, args.output, frame)
    args.json.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(args.output)
    print(args.json)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
