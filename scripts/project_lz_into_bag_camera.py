#!/usr/bin/env python3
"""Project georeferenced landing-zone circles into a ROS 2 bag camera stream."""

from __future__ import annotations

import argparse
import json
import sys
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np
import yaml

from lz_camera_projection import (
    Transform,
    TransformSeries,
    compose,
    fit_planar_rigid,
    latlon_to_utm32,
    load_landing_zones,
    project_optical_points,
    scaled_camera_matrix,
    transform_points,
    zone_ring,
)


def require_ros() -> tuple[object, object, object]:
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        raise RuntimeError(
            "ROS 2 Python modules are unavailable. Run this script in the documented ROS 2 Docker image."
        ) from exc
    return rosbag2_py, deserialize_message, get_message


def open_reader(bag_path: Path):
    rosbag2_py, _, _ = require_ros()
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    return reader


def stamp_ns(message: object, fallback_ns: int) -> int:
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return fallback_ns
    value = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    return value if value > 0 else fallback_ns


def ros_transform(message: object) -> Transform:
    translation = message.transform.translation
    rotation = message.transform.rotation
    return Transform(
        np.array([translation.x, translation.y, translation.z], dtype=np.float64),
        np.array([rotation.x, rotation.y, rotation.z, rotation.w], dtype=np.float64),
    )


def read_pose_and_gps(bag_path: Path, config: dict) -> tuple[dict, list]:
    rosbag2_py, deserialize_message, get_message = require_ros()
    reader = open_reader(bag_path)
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    selected = {config["topics"][key] for key in ("tf", "tf_static", "gps")}
    missing = selected.difference(topic_types)
    if missing:
        raise RuntimeError(f"Bag is missing required topics: {sorted(missing)}")
    reader.set_filter(rosbag2_py.StorageFilter(topics=sorted(selected)))
    message_types = {topic: get_message(topic_types[topic]) for topic in selected}
    dynamic: dict[tuple[str, str], list[tuple[int, Transform]]] = defaultdict(list)
    static: dict[tuple[str, str], Transform] = {}
    gps: list[tuple[int, float, float, float]] = []
    while reader.has_next():
        topic, data, bag_time = reader.read_next()
        if topic not in selected:
            continue
        message = deserialize_message(data, message_types[topic])
        if topic == config["topics"]["gps"]:
            if np.isfinite([message.latitude, message.longitude, message.altitude]).all():
                east, north = latlon_to_utm32(message.latitude, message.longitude)
                gps.append((stamp_ns(message, bag_time), east, north, float(message.altitude)))
            continue
        for item in message.transforms:
            key = (item.header.frame_id.lstrip("/"), item.child_frame_id.lstrip("/"))
            if topic == config["topics"]["tf_static"]:
                static[key] = ros_transform(item)
            else:
                dynamic[key].append((stamp_ns(item, bag_time), ros_transform(item)))
    return {"dynamic": dynamic, "static": static}, gps


def edge_transform(tf_data: dict, parent: str, child: str, timestamp_ns: int, max_gap_s: float) -> Transform:
    key = (parent, child)
    reverse = (child, parent)
    if key in tf_data["static"]:
        return tf_data["static"][key]
    if reverse in tf_data["static"]:
        return tf_data["static"][reverse].inverse()
    cache = tf_data.setdefault("series", {})
    if key in tf_data["dynamic"]:
        if key not in cache:
            cache[key] = TransformSeries(tf_data["dynamic"][key], max_gap_s)
        return cache[key].at(timestamp_ns)
    if reverse in tf_data["dynamic"]:
        if reverse not in cache:
            cache[reverse] = TransformSeries(tf_data["dynamic"][reverse], max_gap_s)
        return cache[reverse].at(timestamp_ns).inverse()
    raise LookupError(f"No TF edge for {parent} -> {child}")


def chain_transform(tf_data: dict, frames: list[str], timestamp_ns: int, max_gap_s: float) -> Transform:
    return compose(
        *(edge_transform(tf_data, parent, child, timestamp_ns, max_gap_s)
          for parent, child in zip(frames[:-1], frames[1:]))
    )


def fit_utm_to_map(tf_data: dict, gps: list, config: dict) -> tuple[Transform, dict]:
    frames = config["frames"]
    max_gap = float(config["synchronization"]["max_tf_gap_s"])
    utm_points = []
    map_points = []
    gps_altitudes = []
    map_altitudes = []
    stride = max(1, len(gps) // 2000)
    for timestamp, east, north, altitude in gps[::stride]:
        try:
            map_to_base = edge_transform(tf_data, frames["map"], frames["base"], timestamp, max_gap)
        except LookupError:
            continue
        utm_points.append((east, north))
        map_points.append(map_to_base.translation[:2])
        gps_altitudes.append(altitude)
        map_altitudes.append(map_to_base.translation[2])
    if len(utm_points) < 10:
        raise RuntimeError(f"Only {len(utm_points)} synchronized GPS/TF pairs were found")
    utm = np.asarray(utm_points)
    local = np.asarray(map_points)
    rotation, translation, residuals = fit_planar_rigid(utm, local)
    yaw = float(np.arctan2(rotation[1, 0], rotation[0, 0]))
    quaternion = np.array([0.0, 0.0, np.sin(yaw / 2.0), np.cos(yaw / 2.0)])
    z_translation = float(np.median(np.asarray(map_altitudes) - np.asarray(gps_altitudes)))
    transform = Transform(np.array([translation[0], translation[1], z_translation]), quaternion)
    baseline = float(np.linalg.norm(np.ptp(utm, axis=0)))
    diagnostics = {
        "synchronized_pairs": len(utm_points),
        "trajectory_baseline_m": baseline,
        "horizontal_rmse_m": float(np.sqrt(np.mean(residuals**2))),
        "horizontal_median_error_m": float(np.median(residuals)),
        "utm_to_map_yaw_deg": float(np.rad2deg(yaw)),
        "map_minus_gnss_altitude_m": z_translation,
    }
    minimum_baseline = float(config["registration"]["minimum_trajectory_baseline_m"])
    maximum_rmse = float(config["registration"]["maximum_horizontal_rmse_m"])
    if baseline < minimum_baseline:
        raise RuntimeError(f"GPS trajectory baseline {baseline:.2f} m is below required {minimum_baseline:.2f} m")
    if diagnostics["horizontal_rmse_m"] > maximum_rmse:
        raise RuntimeError(
            f"GPS/map registration RMSE {diagnostics['horizontal_rmse_m']:.2f} m exceeds {maximum_rmse:.2f} m"
        )
    return transform, diagnostics


def decode_image(message: object) -> np.ndarray:
    channels = {"rgb8": 3, "bgr8": 3, "mono8": 1}.get(message.encoding.lower())
    if channels is None:
        raise ValueError(f"Unsupported image encoding: {message.encoding}")
    packed_width = int(message.width) * channels
    rows = np.frombuffer(message.data, dtype=np.uint8).reshape(int(message.height), int(message.step))
    image = rows[:, :packed_width].reshape(int(message.height), int(message.width), channels)
    if message.encoding.lower() == "rgb8":
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if message.encoding.lower() == "mono8":
        return cv2.cvtColor(image[:, :, 0], cv2.COLOR_GRAY2BGR)
    return image.copy()


def config_transform(item: dict) -> Transform:
    return Transform(
        np.asarray(item["translation_m"], dtype=np.float64),
        np.asarray(item["quaternion_xyzw"], dtype=np.float64),
    )


def draw_zones(frame: np.ndarray, zones: list[dict], map_to_optical: Transform,
               utm_to_map: Transform, camera_matrix: np.ndarray, distortion: np.ndarray,
               vertical_offset: float, alpha: float,
               visible_ranks_out: list[int] | None = None) -> int:
    overlay = frame.copy()
    count = 0
    rendered: list[tuple[np.ndarray, tuple[int, int], tuple[int, int, int], int]] = []
    # Exact CSS colors used by rank_landing_zone_risk.py's browser viewer (BGR here).
    colors = {"safest": (0, 128, 0), "safe": (0, 165, 255), "risky": (0, 0, 255)}
    optical_from_map = map_to_optical.inverse()
    height, width = frame.shape[:2]
    centers_utm = np.asarray(
        [[float(zone["x"]), float(zone["y"]), float(zone["z"]) + vertical_offset]
         for zone in zones],
        dtype=np.float64,
    )
    centers_map = transform_points(utm_to_map, centers_utm)
    centers_optical = transform_points(optical_from_map, centers_map)
    center_pixels, centers_in_front = project_optical_points(centers_optical, camera_matrix, distortion)
    centers_in_frame = (
        centers_in_front
        & (center_pixels[:, 0] >= 0.0) & (center_pixels[:, 0] < width)
        & (center_pixels[:, 1] >= 0.0) & (center_pixels[:, 1] < height)
    )
    visible_indices = np.flatnonzero(centers_in_frame)
    for index in reversed(visible_indices):
        zone = zones[int(index)]
        ring_utm = zone_ring(zone, vertical_offset)
        ring_map = transform_points(utm_to_map, ring_utm)
        ring_optical = transform_points(optical_from_map, ring_map)
        pixels, visible = project_optical_points(ring_optical, camera_matrix, distortion)
        if np.count_nonzero(visible) < len(visible) * 0.8:
            continue
        if not np.isfinite(pixels).all():
            continue
        if (pixels[:, 0].max() < 0 or pixels[:, 0].min() >= width
                or pixels[:, 1].max() < 0 or pixels[:, 1].min() >= height):
            continue
        raster_pixels = pixels.copy()
        raster_pixels[:, 0] = np.clip(raster_pixels[:, 0], -4 * width, 5 * width)
        raster_pixels[:, 1] = np.clip(raster_pixels[:, 1], -4 * height, 5 * height)
        polygon = np.rint(raster_pixels).astype(np.int32)
        color = (144, 238, 144) if int(zone["rank"]) <= 30 else colors.get(str(zone["category"]), (0, 128, 0))
        cv2.fillPoly(overlay, [polygon], color)
        center_array = np.rint(center_pixels[index]).astype(int)
        center_array = np.clip(center_array, [0, 0], [width - 1, height - 1])
        center = (int(center_array[0]), int(center_array[1]))
        rendered.append((polygon, center, color, int(zone["rank"])))
        count += 1
        if visible_ranks_out is not None:
            visible_ranks_out.append(int(zone["rank"]))
    cv2.addWeighted(overlay, alpha, frame, 1.0 - alpha, 0.0, frame)
    for polygon, center, color, rank in rendered:
        cv2.polylines(frame, [polygon], True, color, 3, cv2.LINE_AA)
        cv2.putText(frame, f"LZ {rank}", center, cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (20, 20, 20), 3, cv2.LINE_AA)
        cv2.putText(frame, f"LZ {rank}", center, cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, color, 1, cv2.LINE_AA)
    return count


def process_images(args: argparse.Namespace, config: dict, tf_data: dict, utm_to_map: Transform,
                   zones: list[dict], diagnostics: dict) -> dict:
    _, deserialize_message, get_message = require_ros()
    reader = open_reader(args.bag)
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    image_topic = config["topics"]["image"]
    if image_topic not in topic_types:
        raise RuntimeError(f"Bag has no image topic {image_topic}")
    image_type = get_message(topic_types[image_topic])
    native_matrix = np.asarray(config["camera"]["matrix"], dtype=np.float64)
    distortion = np.asarray(config["camera"]["distortion"], dtype=np.float64)
    native_size = tuple(config["camera"]["calibration_size"])
    optical = config_transform(config["camera"]["mount_to_optical"])
    chain = config["frames"]["camera_chain"]
    max_gap = float(config["synchronization"]["max_tf_gap_s"])
    output_fps = float(args.output_fps or config["output"]["fps"])
    interval_ns = int(1e9 / output_fps)
    first_time = None
    last_written = None
    writer = None
    written = skipped_tf = total_visible = 0
    nadir_tilt_degrees = []
    start_ns = int(args.start_s * 1e9)
    stop_ns = None if args.duration_s <= 0 else start_ns + int(args.duration_s * 1e9)
    while reader.has_next():
        topic, data, bag_time = reader.read_next()
        if topic != image_topic:
            continue
        message = deserialize_message(data, image_type)
        timestamp = stamp_ns(message, bag_time)
        if first_time is None:
            first_time = timestamp
        relative = timestamp - first_time
        if relative < start_ns or (stop_ns is not None and relative > stop_ns):
            continue
        if last_written is not None and timestamp - last_written < interval_ns:
            continue
        frame = decode_image(message)
        if writer is None:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            writer = cv2.VideoWriter(
                str(args.output), cv2.VideoWriter_fourcc(*"mp4v"), output_fps,
                (frame.shape[1], frame.shape[0]),
            )
            if not writer.isOpened():
                raise RuntimeError(f"Could not create video {args.output}")
        camera_matrix = scaled_camera_matrix(native_matrix, native_size, (frame.shape[1], frame.shape[0]))
        try:
            map_to_mount = chain_transform(tf_data, chain, timestamp, max_gap)
        except LookupError:
            skipped_tf += 1
            continue
        map_to_optical = compose(map_to_mount, optical)
        optical_axis_map = map_to_optical.matrix[:3, 2]
        nadir_tilt = float(np.rad2deg(np.arccos(np.clip(-optical_axis_map[2], -1.0, 1.0))))
        nadir_tilt_degrees.append(nadir_tilt)
        if (config["camera"].get("expected_nadir", False)
                and nadir_tilt > float(config["camera"]["maximum_nadir_tilt_deg"])):
            raise RuntimeError(
                f"Camera optical axis is {nadir_tilt:.2f} deg from nadir at t={relative / 1e9:.2f} s; "
                "check mount_to_optical and TF conventions"
            )
        visible = draw_zones(
            frame, zones, map_to_optical, utm_to_map, camera_matrix,
            distortion, float(config["registration"]["lz_to_gnss_vertical_offset_m"]),
            float(config["output"]["fill_alpha"]),
        )
        total_visible += visible
        status = f"{visible} landing zones | t={relative / 1e9:.1f}s"
        cv2.rectangle(frame, (0, 0), (frame.shape[1], 64), (0, 0, 0), -1)
        cv2.putText(frame, status, (18, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (255, 255, 255), 2, cv2.LINE_AA)
        if config["calibration"]["status"] != "measured":
            cv2.putText(frame, "PROVISIONAL CALIBRATION - NOT METRIC VALIDATION", (18, 54),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 210, 255), 2, cv2.LINE_AA)
        writer.write(frame)
        written += 1
        last_written = timestamp
        if args.max_frames > 0 and written >= args.max_frames:
            break
    if writer is not None:
        writer.release()
    if written == 0:
        raise RuntimeError("No output frames were written; inspect time range and TF diagnostics")
    diagnostics.update({
        "output_video": str(args.output),
        "frames_written": written,
        "frames_skipped_missing_tf": skipped_tf,
        "mean_visible_zones": total_visible / written,
        "mean_nadir_tilt_deg": float(np.mean(nadir_tilt_degrees)),
        "maximum_nadir_tilt_deg": float(np.max(nadir_tilt_degrees)),
    })
    return diagnostics


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--landing-zones", type=Path, default=root / "pcds/region_growing_landing_zones_max10000_radius2_5_to3_risk_ranked.csv")
    parser.add_argument("--config", type=Path, default=root / "scripts/config/lz_camera_projection.yaml")
    parser.add_argument("--output", type=Path, default=root / "pcds/lz_camera_projection.mp4")
    parser.add_argument("--diagnostics", type=Path, default=None)
    parser.add_argument("--categories", default="all", help="Comma-separated risk categories, or 'all'")
    parser.add_argument("--max-zones", type=int, default=0, help="Zero loads every matching solution")
    parser.add_argument("--start-s", type=float, default=0.0)
    parser.add_argument("--duration-s", type=float, default=0.0, help="Zero processes to the end")
    parser.add_argument("--output-fps", type=float, default=None)
    parser.add_argument("--max-frames", type=int, default=0, help="Useful for calibration checks")
    parser.add_argument("--allow-provisional-calibration", action="store_true")
    args = parser.parse_args()
    with args.config.open(encoding="utf-8") as handle:
        config = yaml.safe_load(handle)
    if config["calibration"]["status"] != "measured" and not args.allow_provisional_calibration:
        raise RuntimeError(
            "Calibration is marked provisional. Measure the missing values or explicitly pass "
            "--allow-provisional-calibration; provisional output is watermarked."
        )
    categories = None if args.categories.strip().lower() == "all" else {
        item.strip() for item in args.categories.split(",") if item.strip()
    }
    zones = load_landing_zones(args.landing_zones, categories, args.max_zones)
    tf_data, gps = read_pose_and_gps(args.bag, config)
    utm_to_map, diagnostics = fit_utm_to_map(tf_data, gps, config)
    diagnostics.update({"calibration_status": config["calibration"]["status"], "zones_loaded": len(zones)})
    diagnostics = process_images(args, config, tf_data, utm_to_map, zones, diagnostics)
    diagnostics_path = args.diagnostics or args.output.with_suffix(".json")
    diagnostics_path.write_text(json.dumps(diagnostics, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(diagnostics, indent=2))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(1)

