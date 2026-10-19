#!/usr/bin/env python3
"""Replay a ROS 2 bag and stream synchronized state to the LZ browser viewer."""

from __future__ import annotations

import argparse
import json
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np
import yaml

SCRIPTS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPTS))

from lz_camera_projection import (  # noqa: E402
    Transform, compose, load_landing_zones, scaled_camera_matrix, transform_points, zone_ring,
)
from project_lz_into_bag_camera import (  # noqa: E402
    chain_transform, config_transform, decode_image, draw_zones, fit_utm_to_map,
    open_reader, read_pose_and_gps, require_ros, stamp_ns,
)


class ReplayState:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.pause = threading.Event()
        self.restart = threading.Event()
        self.rate = 1.0
        self.jpeg = b""
        self.data = {"seq": 0, "status": "Preparing bag", "trajectory": {"x": [], "y": [], "z": []}}

    def snapshot(self) -> dict:
        with self.lock:
            return json.loads(json.dumps(self.data))


def orthometric_from_map(utm_to_map: Transform, vertical_offset: float) -> Transform:
    ortho_to_gnss = Transform(np.array([0.0, 0.0, vertical_offset]), np.array([0.0, 0.0, 0.0, 1.0]))
    return compose(utm_to_map, ortho_to_gnss).inverse()


def camera_footprint(ortho_to_optical: Transform, camera_matrix: np.ndarray, distortion: np.ndarray,
                     width: int, height: int, ground_z: float) -> tuple[list[list[float]], dict[str, list[float]]]:
    camera = ortho_to_optical.translation
    corners = np.array([[0.0, 0.0], [width - 1.0, 0.0],
                        [width - 1.0, height - 1.0], [0.0, height - 1.0]], dtype=np.float64)
    normalized = cv2.undistortPoints(corners.reshape(-1, 1, 2), camera_matrix, distortion).reshape(-1, 2)
    rays_camera = np.column_stack((normalized, np.ones(4)))
    rays_world = (ortho_to_optical.matrix[:3, :3] @ rays_camera.T).T
    footprint = []
    for ray in rays_world:
        if ray[2] >= -1e-6:
            return [], {"x": [], "y": [], "z": []}
        scale = (ground_z - camera[2]) / ray[2]
        footprint.append((camera + scale * ray).tolist())
    closed = footprint + [footprint[0]]
    frustum_x: list[float | None] = []
    frustum_y: list[float | None] = []
    frustum_z: list[float | None] = []
    for point in footprint:
        frustum_x.extend([float(camera[0]), point[0], None])
        frustum_y.extend([float(camera[1]), point[1], None])
        frustum_z.extend([float(camera[2]), point[2], None])
    return closed, {"x": frustum_x, "y": frustum_y, "z": frustum_z}


def frustum_visible_ranks(zones: list[dict], map_to_optical: Transform, utm_to_map: Transform,
                          camera_matrix: np.ndarray, distortion: np.ndarray,
                          width: int, height: int, vertical_offset: float) -> list[int]:
    """Independently classify renderable zone footprints against the 3D frustum."""

    coefficients = np.zeros(5, dtype=np.float64)
    supplied = np.asarray(distortion, dtype=np.float64).reshape(-1)
    coefficients[:min(5, len(supplied))] = supplied[:5]
    k1, k2, p1, p2, k3 = coefficients
    centers_utm = np.asarray(
        [[float(zone["x"]), float(zone["y"]), float(zone["z"]) + vertical_offset] for zone in zones],
        dtype=np.float64,
    )
    centers_map = transform_points(utm_to_map, centers_utm)
    optical_from_map = map_to_optical.inverse()
    centers_optical = transform_points(optical_from_map, centers_map)
    ranks = []
    for zone, center in zip(zones, centers_optical):
        if center[2] <= 0.1:
            continue
        # Independent Brown-Conrady projection for the center. This deliberately
        # does not call draw_zones/project_optical_points or consume their IDs.
        x = float(center[0] / center[2])
        y = float(center[1] / center[2])
        r2 = x * x + y * y
        radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2
        distorted_x = x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x)
        distorted_y = y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y
        pixel = camera_matrix @ np.array([distorted_x, distorted_y, 1.0])
        if not (0.0 <= pixel[0] < width and 0.0 <= pixel[1] < height):
            continue
        ring_utm = zone_ring(zone, vertical_offset_m=vertical_offset, samples=72)
        ring_map = transform_points(utm_to_map, ring_utm)
        ring_optical = transform_points(optical_from_map, ring_map)
        # Match the renderer's declared partial-footprint policy without sharing
        # its projected pixels or selected IDs. This permits a circle crossing
        # the near plane only when at least 80% remains geometrically valid.
        if np.count_nonzero(ring_optical[:, 2] > 0.1) < len(ring_optical) * 0.8:
            continue
        ranks.append(int(zone["rank"]))
    return sorted(ranks)


def replay_loop(args: argparse.Namespace, state: ReplayState) -> None:
    with args.projection_config.open(encoding="utf-8") as handle:
        config = yaml.safe_load(handle)
    zones = load_landing_zones(args.zones, None, 0)
    zones_by_rank = {int(zone["rank"]): zone for zone in zones}
    tf_data, gps = read_pose_and_gps(args.bag, config)
    utm_to_map, registration = fit_utm_to_map(tf_data, gps, config)
    vertical_offset = float(config["registration"]["lz_to_gnss_vertical_offset_m"])
    map_to_ortho = orthometric_from_map(utm_to_map, vertical_offset)
    ground_z = float(np.median([float(zone["z"]) for zone in zones]))
    _, deserialize_message, get_message = require_ros()
    while True:
        reader = open_reader(args.bag)
        topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
        image_topic = config["topics"]["image"]
        image_type = get_message(topic_types[image_topic])
        first_stamp = last_publish = None
        wall_start = time.monotonic()
        trajectory = {"x": [], "y": [], "z": []}
        frame_number = 0
        checked_frames = 0
        mismatch_frames = 0
        mismatch_examples: list[dict[str, object]] = []
        with state.lock:
            state.data.update({"status": "Playing", "registration": registration, "trajectory": trajectory})
        while reader.has_next():
            if state.restart.is_set():
                state.restart.clear()
                break
            while state.pause.is_set():
                time.sleep(0.05)
                wall_start += 0.05
            topic, raw, bag_time = reader.read_next()
            if topic != image_topic:
                continue
            message = deserialize_message(raw, image_type)
            timestamp = stamp_ns(message, bag_time)
            if first_stamp is None:
                first_stamp = timestamp
            if last_publish is not None and timestamp - last_publish < int(1e9 / args.publish_fps):
                continue
            desired = (timestamp - first_stamp) / 1e9 / state.rate
            delay = desired - (time.monotonic() - wall_start)
            if delay > 0:
                time.sleep(min(delay, 1.0))
            frame = decode_image(message)
            native_k = np.asarray(config["camera"]["matrix"], dtype=np.float64)
            camera_matrix = scaled_camera_matrix(native_k, tuple(config["camera"]["calibration_size"]), (frame.shape[1], frame.shape[0]))
            map_to_mount = chain_transform(tf_data, config["frames"]["camera_chain"], timestamp,
                                            float(config["synchronization"]["max_tf_gap_s"]))
            map_to_optical = compose(map_to_mount, config_transform(config["camera"]["mount_to_optical"]))
            image_visible_ranks: list[int] = []
            visible = draw_zones(
                frame, zones, map_to_optical, utm_to_map, camera_matrix,
                np.asarray(config["camera"]["distortion"]),
                vertical_offset, 0.25,
                visible_ranks_out=image_visible_ranks,
            )
            image_visible_ranks = sorted(image_visible_ranks)
            frustum_ranks = frustum_visible_ranks(
                zones, map_to_optical, utm_to_map, camera_matrix,
                np.asarray(config["camera"]["distortion"]), frame.shape[1], frame.shape[0],
                vertical_offset,
            )
            image_only = sorted(set(image_visible_ranks) - set(frustum_ranks))
            frustum_only = sorted(set(frustum_ranks) - set(image_visible_ranks))
            sets_match = not image_only and not frustum_only and visible == len(image_visible_ranks)
            checked_frames += 1
            if not sets_match:
                mismatch_frames += 1
                if len(mismatch_examples) < 10:
                    mismatch_examples.append({
                        "bag_time_s": (timestamp - first_stamp) / 1e9,
                        "image_only": image_only,
                        "frustum_only": frustum_only,
                    })
            visible_zones = [zones_by_rank[rank] for rank in frustum_ranks]
            visible_lzs: dict[str, list[float | None] | list[int]] = {
                "x": [], "y": [], "z": [], "rank": frustum_ranks,
            }
            for zone in visible_zones:
                ring = zone_ring(zone, vertical_offset_m=vertical_offset, samples=48)
                ring = np.vstack((ring, ring[0]))
                for column, axis in enumerate(("x", "y", "z")):
                    visible_lzs[axis].extend(ring[:, column].tolist() + [None])
            ortho_to_optical = compose(map_to_ortho, map_to_optical)
            map_to_base = chain_transform(tf_data, [config["frames"]["map"], config["frames"]["base"]], timestamp,
                                          float(config["synchronization"]["max_tf_gap_s"]))
            uav = transform_points(map_to_ortho, map_to_base.translation[None, :])[0]
            if not trajectory["x"] or np.linalg.norm(uav - np.array([trajectory["x"][-1], trajectory["y"][-1], trajectory["z"][-1]])) > 0.05:
                trajectory["x"].append(float(uav[0])); trajectory["y"].append(float(uav[1])); trajectory["z"].append(float(uav[2]))
            footprint_points, frustum = camera_footprint(
                ortho_to_optical, camera_matrix, np.asarray(config["camera"]["distortion"]),
                frame.shape[1], frame.shape[0], ground_z,
            )
            footprint = {axis: [point[index] for point in footprint_points] for index, axis in enumerate(("x", "y", "z"))}
            ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 82])
            if not ok:
                continue
            frame_number += 1
            with state.lock:
                state.jpeg = encoded.tobytes()
                state.data = {
                    "seq": state.data.get("seq", 0) + 1, "status": "Playing",
                    "bag_time_s": (timestamp - first_stamp) / 1e9, "frame_number": frame_number,
                    "visible_zones": visible,
                    "image_visible_zone_ranks": image_visible_ranks,
                    "frustum_visible_zone_ranks": frustum_ranks,
                    "image_only_zone_ranks": image_only,
                    "frustum_only_zone_ranks": frustum_only,
                    "checked_frames": checked_frames,
                    "mismatch_frames": mismatch_frames,
                    "mismatch_examples": mismatch_examples.copy(),
                    "projection_sets_match": sets_match, "visible_lzs": visible_lzs,
                    "camera_position": ortho_to_optical.translation.tolist(),
                    "uav_position": uav.tolist(),
                    "trajectory": {axis: values.copy() for axis, values in trajectory.items()},
                    "footprint": footprint, "frustum": frustum,
                }
            last_publish = timestamp
        else:
            with state.lock:
                state.data["status"] = "Replay complete"
            state.pause.set()
            while not state.restart.is_set():
                time.sleep(0.1)
            state.restart.clear()
            state.pause.clear()


def handler_factory(args: argparse.Namespace, state: ReplayState):
    class Handler(BaseHTTPRequestHandler):
        def send_bytes(self, body: bytes, content_type: str) -> None:
            self.send_response(200); self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body))); self.send_header("Cache-Control", "no-store")
            self.end_headers(); self.wfile.write(body)

        def do_GET(self) -> None:
            path = urlparse(self.path).path
            if path == "/":
                self.send_bytes(args.viewer.read_bytes(), "text/html; charset=utf-8")
            elif path == "/api/state":
                self.send_bytes(json.dumps(state.snapshot()).encode(), "application/json")
            elif path == "/api/frame.jpg":
                with state.lock:
                    jpeg = state.jpeg
                self.send_bytes(jpeg, "image/jpeg")
            else:
                self.send_error(404)

        def do_POST(self) -> None:
            query = parse_qs(urlparse(self.path).query)
            command = query.get("cmd", [""])[0]
            if command == "pause": state.pause.set()
            elif command == "play": state.pause.clear()
            elif command == "restart": state.pause.clear(); state.restart.set()
            elif command == "rate": state.rate = max(0.1, min(8.0, float(query.get("value", ["1"])[0])))
            self.send_bytes(b"{}", "application/json")

        def log_message(self, format: str, *values: object) -> None:
            return
    return Handler


def main() -> int:
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--viewer", type=Path, default=root / "pcds/region_growing_landing_zones_live_bag_replay.html")
    parser.add_argument("--zones", type=Path, default=root / "pcds/region_growing_landing_zones_max10000_radius2_5_to3_risk_ranked.csv")
    parser.add_argument("--projection-config", type=Path, default=root / "scripts/config/lz_camera_projection.yaml")
    parser.add_argument("--host", default="0.0.0.0"); parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--publish-fps", type=float, default=8.0)
    args = parser.parse_args()
    if not args.viewer.exists():
        raise FileNotFoundError(f"Prepare the viewer first: {args.viewer}")
    state = ReplayState()
    threading.Thread(target=replay_loop, args=(args, state), daemon=True).start()
    server = ThreadingHTTPServer((args.host, args.port), handler_factory(args, state))
    print(f"Replay viewer: http://127.0.0.1:{args.port}")
    server.serve_forever()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
