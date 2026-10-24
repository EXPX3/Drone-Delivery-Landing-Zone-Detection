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


def canonical_class(prompt: str) -> str:
    name = prompt.lower()
    if "person" in name or "human" in name:
        return "person"
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
