#!/usr/bin/env python3
"""Open-vocabulary image segmentation and image-to-LiDAR label transfer."""

from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path

import cv2
import numpy as np


CLASS_COLORS_BGR = {
    "person": (255, 0, 255),
    "cargo_box": (0, 255, 255),
    "vehicle": (255, 96, 0),
}
CLASS_COLORS_RGB = {name: color[::-1] for name, color in CLASS_COLORS_BGR.items()}
STATIC_CLASSES = frozenset({"cargo_box", "vehicle"})


def canonical_class(prompt: str) -> str:
    name = prompt.lower()
    if "person" in name or "human" in name:
        return "person"
    if name.strip() in {"white rectangular obstacle", "black rectangular obstacle"}:
        return "cargo_box"
    if "box" in name or "crate" in name or "package" in name:
        return "cargo_box"
    if name.strip() in {"car", "vehicle", "truck"}:
        return "vehicle"
    raise ValueError(f"Unmapped semantic prompt: {prompt}")


@dataclass
class SemanticResult:
    class_names: list[str]
    confidences: np.ndarray
    masks: np.ndarray


class SemanticVoxelMemory:
    """Persistent class labels with hysteresis against contradictory detections."""

    def __init__(self, resolution_m: float = 0.20, contradiction_hits: int = 2,
                 confirmation_hits: int = 2) -> None:
        self.resolution_m = float(resolution_m)
        self.contradiction_hits = int(contradiction_hits)
        self.confirmation_hits = int(confirmation_hits)
        self._labels: dict[tuple[int, int, int], str] = {}
        self._hits: dict[tuple[int, int, int], int] = {}
        self._contradictions: dict[tuple[int, int, int], tuple[str, int]] = {}

    def update(self, points: np.ndarray, names: np.ndarray) -> None:
        # One camera/LiDAR association is one observation. Multiple returns in
        # the same voxel during that association must not fake temporal support.
        observations: dict[tuple[int, int, int], str] = {}
        for point, name_value in zip(np.asarray(points), np.asarray(names, dtype=object)):
            name = str(name_value)
            key = tuple(np.floor(point / self.resolution_m).astype(np.int64))
            observations.setdefault(key, name)
        for key, name in observations.items():
            previous = self._labels.get(key)
            if previous is None or previous == name:
                self._labels[key] = name
                self._hits[key] = self._hits.get(key, 0) + 1
                self._contradictions.pop(key, None)
                continue
            candidate, hits = self._contradictions.get(key, (name, 0))
            hits = hits + 1 if candidate == name else 1
            if hits >= self.contradiction_hits:
                self._labels[key] = name
                self._contradictions.pop(key, None)
            else:
                self._contradictions[key] = (name, hits)

    def point_cloud(self) -> tuple[np.ndarray, np.ndarray]:
        if not self._labels:
            return np.empty((0, 3), dtype=np.float64), np.empty((0, 3), dtype=np.uint8)
        confirmed = [key for key in self._labels if self._hits.get(key, 0) >= self.confirmation_hits]
        if not confirmed:
            return np.empty((0, 3), dtype=np.float64), np.empty((0, 3), dtype=np.uint8)
        keys = np.asarray(confirmed, dtype=np.float64)
        points = (keys + 0.5) * self.resolution_m
        colors = np.asarray(
            [CLASS_COLORS_RGB[self._labels[key]] for key in confirmed], dtype=np.uint8,
        )
        return points, colors


class VisionProposalTracker:
    """Short-term 3D display memory for camera-only ground projections."""

    def __init__(self, association_radius_m: float = 2.0,
                 retention_s: dict[str, float] | None = None,
                 smoothing: float = 0.35) -> None:
        self.association_radius_m = float(association_radius_m)
        self.retention_s = retention_s or {
            "person": 2.0, "cargo_box": 3.0, "vehicle": 3.0,
        }
        self.smoothing = float(smoothing)
        self.proposals: list[dict] = []
        self._next_id = 1

    def update(self, detections: list[dict], timestamp_ns: int) -> list[dict]:
        used: set[int] = set()
        for detection in detections:
            center = np.asarray(detection["center"], dtype=np.float64)
            candidates = [
                (float(np.linalg.norm(np.asarray(item["center"])[:2] - center[:2])), index)
                for index, item in enumerate(self.proposals)
                if index not in used and item["class_name"] == detection["class_name"]
            ]
            distance, index = min(candidates, default=(np.inf, -1))
            if distance <= self.association_radius_m:
                item = self.proposals[index]
                item["center"] = (
                    (1.0 - self.smoothing) * np.asarray(item["center"])
                    + self.smoothing * center
                )
                item["covariance_xy"] = (
                    (1.0 - self.smoothing) * np.asarray(item["covariance_xy"])
                    + self.smoothing * np.asarray(detection["covariance_xy"])
                )
                item["confidence"] = float(detection["confidence"])
                item["maximum_confidence"] = max(
                    float(item["maximum_confidence"]), float(detection["confidence"]),
                )
                item["observations"] += 1
                item["last_seen_ns"] = int(timestamp_ns)
                used.add(index)
            else:
                item = dict(detection)
                item.update({
                    "proposal_id": self._next_id,
                    "maximum_confidence": float(detection["confidence"]),
                    "observations": 1,
                    "last_seen_ns": int(timestamp_ns),
                })
                self.proposals.append(item)
                used.add(len(self.proposals) - 1)
                self._next_id += 1
        return self.active(timestamp_ns)

    def active(self, timestamp_ns: int) -> list[dict]:
        self.proposals = [
            item for item in self.proposals
            if (timestamp_ns - int(item["last_seen_ns"])) / 1e9
            <= float(self.retention_s.get(item["class_name"], 2.0))
        ]
        return self.proposals


@dataclass
class SemanticObjectTrack:
    track_id: int
    class_name: str
    centroid: np.ndarray
    points: np.ndarray
    observations: int = 1
    last_frame: int = 0
    confidence: float = 0.0
    centers: list[np.ndarray] | None = None
    display_id: int | None = None
    accumulated_points: np.ndarray | None = None

    def __post_init__(self) -> None:
        if self.centers is None:
            self.centers = [self.centroid.copy()]
        if self.accumulated_points is None:
            self.accumulated_points = self.points.copy()


class SemanticObjectTracker:
    """Fuse sparse same-class 3D clusters into compact object tracks."""

    def __init__(self, cluster_radius_m: float = 0.75, association_radius_m: float = 1.2,
                 confirmation_frames: int = 3, smoothing: float = 0.35,
                 maximum_age_frames: int = 90, history_size: int = 60,
                 duplicate_merge_radius_m: float = 1.5,
                 confirmed_duplicate_merge_radius_m: float = 2.5,
                 static_voxel_size_m: float = 0.10) -> None:
        self.cluster_radius_m = float(cluster_radius_m)
        self.association_radius_m = float(association_radius_m)
        self.confirmation_frames = int(confirmation_frames)
        self.smoothing = float(smoothing)
        self.maximum_age_frames = int(maximum_age_frames)
        self.history_size = int(history_size)
        self.duplicate_merge_radius_m = float(duplicate_merge_radius_m)
        self.confirmed_duplicate_merge_radius_m = float(
            confirmed_duplicate_merge_radius_m
        )
        if self.confirmed_duplicate_merge_radius_m < self.duplicate_merge_radius_m:
            raise ValueError("Confirmed duplicate merge radius cannot be smaller")
        self.static_voxel_size_m = float(static_voxel_size_m)
        self.tracks: list[SemanticObjectTrack] = []
        self._next_id = 1
        self._next_display_id: dict[str, int] = {}

    def _confirm_display_id(self, track: SemanticObjectTrack) -> None:
        if track.observations < self.confirmation_frames or track.display_id is not None:
            return
        next_id = self._next_display_id.get(track.class_name, 1)
        track.display_id = next_id
        self._next_display_id[track.class_name] = next_id + 1

    def _clusters(self, points: np.ndarray, names: np.ndarray) -> list[tuple[str, np.ndarray]]:
        output: list[tuple[str, np.ndarray]] = []
        for class_name in set(names):
            class_points = points[names == class_name]
            remaining = set(range(len(class_points)))
            while remaining:
                component = {remaining.pop()}
                frontier = list(component)
                while frontier:
                    current = frontier.pop()
                    neighbors = {
                        index for index in remaining
                        if np.linalg.norm(class_points[index] - class_points[current])
                        <= self.cluster_radius_m
                    }
                    remaining.difference_update(neighbors)
                    component.update(neighbors)
                    frontier.extend(neighbors)
                output.append((str(class_name), class_points[sorted(component)]))
        return output

    def update(self, points: np.ndarray, names: np.ndarray, frame_index: int = 0,
               confidences: np.ndarray | None = None) -> None:
        points = np.asarray(points, dtype=np.float64)
        names = np.asarray(names, dtype=object)
        if not len(points):
            self.expire(frame_index)
            return
        if confidences is None:
            confidences = np.ones(len(points), dtype=np.float64)
        else:
            confidences = np.asarray(confidences, dtype=np.float64)
        used_tracks: set[int] = set()
        for class_name, cluster in self._clusters(points, names):
            center = np.median(cluster, axis=0)
            cluster_confidence = float(np.max(confidences[names == class_name]))
            candidates = [
                (float(np.linalg.norm(track.centroid - center)), index)
                for index, track in enumerate(self.tracks)
                if index not in used_tracks and track.class_name == class_name
            ]
            distance, index = min(candidates, default=(np.inf, -1))
            if distance <= self.association_radius_m:
                track = self.tracks[index]
                track.centroid = (1.0 - self.smoothing) * track.centroid + self.smoothing * center
                track.points = cluster.copy()
                track.observations += 1
                track.last_frame = int(frame_index)
                track.confidence = max(track.confidence, cluster_confidence)
                track.centers.append(center.copy())
                track.centers = track.centers[-self.history_size:]
                if class_name in STATIC_CLASSES:
                    track.accumulated_points = self._voxel_union(
                        track.accumulated_points, cluster,
                    )
                else:
                    track.accumulated_points = cluster.copy()
                self._confirm_display_id(track)
                used_tracks.add(index)
            else:
                track = SemanticObjectTrack(
                    self._next_id, class_name, center.copy(), cluster.copy(),
                    last_frame=int(frame_index), confidence=cluster_confidence,
                )
                self._confirm_display_id(track)
                self.tracks.append(track)
                used_tracks.add(len(self.tracks) - 1)
                self._next_id += 1
        self._merge_duplicate_tracks()
        self.expire(frame_index)

    def _voxel_union(self, first: np.ndarray, second: np.ndarray) -> np.ndarray:
        values = np.vstack((np.asarray(first), np.asarray(second)))
        keys = np.floor(values / self.static_voxel_size_m).astype(np.int64)
        _, indices = np.unique(keys, axis=0, return_index=True)
        return values[np.sort(indices)]

    def _merge_duplicate_tracks(self) -> None:
        """Fuse same-object tracks created by camera/LiDAR projection jitter."""
        changed = True
        while changed:
            changed = False
            for first_index, first in enumerate(self.tracks):
                for second_index in range(first_index + 1, len(self.tracks)):
                    second = self.tracks[second_index]
                    if first.class_name != second.class_name:
                        continue
                    confirmed = (first.observations >= self.confirmation_frames
                                 or second.observations >= self.confirmation_frames)
                    merge_radius = (self.confirmed_duplicate_merge_radius_m if confirmed
                                    else self.duplicate_merge_radius_m)
                    if np.linalg.norm(first.centroid[:2] - second.centroid[:2]) > merge_radius:
                        continue
                    keep, merge = ((first, second) if first.track_id < second.track_id
                                   else (second, first))
                    combined = np.vstack((keep.centers, merge.centers))[-self.history_size:]
                    keep.centers = [value.copy() for value in combined]
                    keep.centroid = np.median(combined, axis=0)
                    keep.observations += merge.observations
                    if merge.last_frame >= keep.last_frame:
                        keep.points = merge.points.copy()
                    keep.last_frame = max(keep.last_frame, merge.last_frame)
                    keep.confidence = max(keep.confidence, merge.confidence)
                    keep.accumulated_points = self._voxel_union(
                        keep.accumulated_points, merge.accumulated_points,
                    )
                    keep.display_id = keep.display_id or merge.display_id
                    self._confirm_display_id(keep)
                    self.tracks = [track for track in self.tracks if track is not merge]
                    changed = True
                    break
                if changed:
                    break

    def expire(self, frame_index: int) -> None:
        if frame_index <= 0 or self.maximum_age_frames <= 0:
            return
        self.tracks = [
            track for track in self.tracks
            if (track.class_name in STATIC_CLASSES
                and track.observations >= self.confirmation_frames)
            or frame_index - track.last_frame <= self.maximum_age_frames
        ]

    @staticmethod
    def track_color(track: SemanticObjectTrack) -> np.ndarray:
        base = np.asarray(CLASS_COLORS_RGB[track.class_name], dtype=np.float64)
        factors = (1.0, 0.72, 0.52, 0.86, 0.62)
        factor = factors[(track.track_id - 1) % len(factors)]
        # Retain the semantic hue while making same-class instances distinguishable.
        return np.clip(base * factor + (255.0 - base) * (1.0 - factor) * 0.25,
                       0, 255).astype(np.uint8)

    def confirmed_tracks(self) -> list[SemanticObjectTrack]:
        return [track for track in self.tracks
                if track.observations >= self.confirmation_frames]

    def point_cloud(self) -> tuple[np.ndarray, np.ndarray]:
        visible = self.tracks
        if not visible:
            return np.empty((0, 3), dtype=np.float64), np.empty((0, 3), dtype=np.uint8)
        clouds, colors = [], []
        for track in visible:
            source = (track.accumulated_points if track.class_name in STATIC_CLASSES
                      else track.points)
            points = np.vstack((source, track.centroid[None, :]))
            clouds.append(points)
            color = (np.asarray(CLASS_COLORS_RGB["cargo_box"], dtype=np.uint8)
                     if track.class_name == "cargo_box" else self.track_color(track))
            colors.append(np.tile(color, (len(points), 1)))
        return np.vstack(clouds), np.vstack(colors).astype(np.uint8)

    def summaries(self) -> list[dict]:
        output = []
        for track in self.tracks:
            centers = np.asarray(track.centers, dtype=np.float64)
            center = np.median(centers, axis=0)
            radial = np.linalg.norm(centers[:, :2] - center[:2], axis=1)
            covariance_xy = (np.cov(centers[:, :2], rowvar=False)
                             if len(centers) > 1 else np.eye(2) * 0.01)
            output.append({
                "track_id": track.track_id,
                "display_id": track.display_id,
                "class_name": track.class_name,
                "motion_model": ("static" if track.class_name in STATIC_CLASSES else "dynamic"),
                "observations": track.observations,
                "confirmed": track.observations >= self.confirmation_frames,
                "centroid": track.centroid.tolist(),
                "latest_point_count": len(track.points),
                "last_frame": track.last_frame,
                "confidence": track.confidence,
                "empirical_std_xyz_m": np.std(centers, axis=0).tolist(),
                "empirical_covariance_xy_m2": covariance_xy.tolist(),
                "empirical_radial_p95_m": float(np.percentile(radial, 95)),
            })
        return output


def compact_supported_clusters(points: np.ndarray, class_name: str,
                               cluster_radius_m: float, minimum_points: int,
                               maximum_radial_p95_m: float) -> list[np.ndarray]:
    """Return physically compact connected components from one image detection."""
    values = np.asarray(points, dtype=np.float64)
    if not len(values):
        return []
    tracker = SemanticObjectTracker(cluster_radius_m=cluster_radius_m)
    names = np.full(len(values), class_name, dtype=object)
    accepted = []
    for _, cluster in tracker._clusters(values, names):
        if len(cluster) < minimum_points:
            continue
        center = np.median(cluster[:, :2], axis=0)
        radial = np.linalg.norm(cluster[:, :2] - center, axis=1)
        if float(np.percentile(radial, 95)) <= maximum_radial_p95_m:
            accepted.append(cluster)
    return accepted


class OpenVocabularySegmenter:
    """YOLOE promptable segmenter, loaded lazily to keep tests CPU-only."""

    def __init__(self, weights: str, prompts: list[str], confidence: float = 0.12,
                 image_size: int = 960, device: str = "0") -> None:
        from ultralytics import YOLOE

        self.prompts = prompts
        self.confidence = confidence
        self.image_size = image_size
        self.device = device
        weights_path = Path(weights).resolve()
        self.model = YOLOE(str(weights_path))
        # Ultralytics resolves the text encoder by filename in the process CWD.
        # Keep all large model assets together without copying them into the repo root.
        previous_directory = Path.cwd()
        try:
            os.chdir(weights_path.parent)
            self.model.set_classes(prompts)
        finally:
            os.chdir(previous_directory)

    def predict(self, frame: np.ndarray) -> SemanticResult:
        result = self.model.predict(
            frame, imgsz=self.image_size, conf=self.confidence,
            device=self.device, verbose=False,
        )[0]
        height, width = frame.shape[:2]
        masks: list[np.ndarray] = []
        names: list[str] = []
        confidences: list[float] = []
        if result.boxes is None or result.masks is None:
            return SemanticResult([], np.empty(0), np.empty((0, height, width), dtype=bool))
        classes = result.boxes.cls.detach().cpu().numpy().astype(int)
        scores = result.boxes.conf.detach().cpu().numpy()
        for class_index, score, polygon_set in zip(classes, scores, result.masks.xy):
            mask = np.zeros((height, width), dtype=np.uint8)
            polygon = np.rint(polygon_set).astype(np.int32)
            if len(polygon) >= 3:
                cv2.fillPoly(mask, [polygon], 1)
                masks.append(mask.astype(bool))
                names.append(canonical_class(self.prompts[class_index]))
                confidences.append(float(score))
        data = np.asarray(masks, dtype=bool) if masks else np.empty((0, height, width), dtype=bool)
        return SemanticResult(names, np.asarray(confidences), data)


def colorize_frame(frame: np.ndarray, result: SemanticResult, alpha: float = 0.42) -> np.ndarray:
    output = frame.copy()
    overlay = frame.copy()
    for name, score, mask in zip(result.class_names, result.confidences, result.masks):
        color = CLASS_COLORS_BGR[name]
        overlay[mask] = color
        contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(output, contours, -1, color, 3)
        if contours:
            x, y, _, _ = cv2.boundingRect(max(contours, key=cv2.contourArea))
            cv2.putText(output, f"{name} {score:.2f}", (x, max(24, y - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.62, color, 2, cv2.LINE_AA)
    return cv2.addWeighted(overlay, alpha, output, 1.0 - alpha, 0.0)


def filter_semantic_result(result: SemanticResult, minimum_confidence: dict,
                           maximum_mask_fraction: dict) -> SemanticResult:
    """Keep only image detections eligible for physical 3D verification."""
    keep = np.asarray([
        score >= float(minimum_confidence.get(name, 1.0))
        and np.count_nonzero(mask) / mask.size
        <= float(maximum_mask_fraction.get(name, 1.0))
        for name, score, mask in zip(result.class_names, result.confidences, result.masks)
    ], dtype=bool)
    return SemanticResult(
        [name for name, selected in zip(result.class_names, keep) if selected],
        result.confidences[keep], result.masks[keep],
    )


def labels_at_pixels(result: SemanticResult, pixels: np.ndarray, valid: np.ndarray) -> np.ndarray:
    """Return class indices (1..N) for projected pixels; 0 means unlabeled."""
    labels = np.zeros(len(pixels), dtype=np.uint8)
    if not len(result.masks):
        return labels
    height, width = result.masks.shape[1:]
    valid = np.asarray(valid, dtype=bool) & np.isfinite(pixels).all(axis=1)
    indices = np.flatnonzero(valid)
    xy = np.rint(pixels[indices]).astype(int)
    inside = (xy[:, 0] >= 0) & (xy[:, 0] < width) & (xy[:, 1] >= 0) & (xy[:, 1] < height)
    indices, xy = indices[inside], xy[inside]
    # Higher-confidence masks win in overlap regions.
    for detection in np.argsort(result.confidences):
        selected = result.masks[detection, xy[:, 1], xy[:, 0]]
        labels[indices[selected]] = detection + 1
    return labels
