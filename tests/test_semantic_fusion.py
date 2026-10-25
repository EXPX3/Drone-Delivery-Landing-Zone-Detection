import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from semantic_lz.semantic_fusion import (
    SemanticObjectTracker, SemanticResult, SemanticVoxelMemory,
    canonical_class, labels_at_pixels,
)


class SemanticFusionTests(unittest.TestCase):
    def test_canonical_classes_cover_safety_obstacles(self):
        self.assertEqual(canonical_class("person"), "person")
        self.assertEqual(canonical_class("cardboard box"), "cargo_box")
        self.assertEqual(canonical_class("white rectangular obstacle"), "cargo_box")
        self.assertEqual(canonical_class("car"), "vehicle")

    def test_projected_pixels_receive_mask_label(self):
        mask = np.zeros((1, 8, 8), dtype=bool)
        mask[0, 2:6, 2:6] = True
        result = SemanticResult(["person"], np.array([0.9]), mask)
        pixels = np.array([[3.0, 4.0], [7.0, 7.0], [3.0, 4.0]])
        valid = np.array([True, True, False])
        self.assertEqual(labels_at_pixels(result, pixels, valid).tolist(), [1, 0, 0])

    def test_semantic_memory_requires_confirmed_contradiction(self):
        memory = SemanticVoxelMemory(resolution_m=0.2, contradiction_hits=2, confirmation_hits=2)
        point = np.array([[1.01, 2.01, 3.01]])
        memory.update(point, np.array(["person"], dtype=object))
        memory.update(point, np.array(["person"], dtype=object))
        memory.update(point, np.array(["cargo_box"], dtype=object))
        _, colors = memory.point_cloud()
        self.assertEqual(colors.tolist(), [[255, 0, 255]])
        memory.update(point, np.array(["cargo_box"], dtype=object))
        _, colors = memory.point_cloud()
        self.assertEqual(colors.tolist(), [[255, 255, 0]])

    def test_one_frame_cannot_confirm_with_duplicate_returns(self):
        memory = SemanticVoxelMemory(resolution_m=0.2, confirmation_hits=2)
        points = np.array([[1.01, 2.01, 3.01], [1.02, 2.02, 3.02]])
        names = np.array(["person", "person"], dtype=object)
        memory.update(points, names)
        visible, _ = memory.point_cloud()
        self.assertEqual(len(visible), 0)
        memory.update(points[:1], names[:1])
        visible, _ = memory.point_cloud()
        self.assertEqual(len(visible), 1)

    def test_object_tracker_associates_neighboring_returns_without_spreading(self):
        tracker = SemanticObjectTracker(
            cluster_radius_m=0.5, association_radius_m=1.0,
            confirmation_frames=3, smoothing=0.5,
        )
        names = np.array(["cargo_box", "cargo_box"], dtype=object)
        tracker.update(np.array([[0.0, 0.0, 0.5], [0.1, 0.0, 0.5]]), names)
        tracker.update(np.array([[0.25, 0.0, 0.5], [0.35, 0.0, 0.5]]), names)
        points, _ = tracker.point_cloud()
        self.assertEqual(len(points), 0)
        latest = np.array([[0.45, 0.0, 0.5], [0.55, 0.0, 0.5]])
        tracker.update(latest, names)
        points, colors = tracker.point_cloud()
        self.assertEqual(len(tracker.tracks), 1)
        self.assertEqual(len(points), 3)
        self.assertTrue(np.all(points[:2, 0] >= 0.45))
        self.assertEqual(colors[0].tolist(), [255, 255, 0])


if __name__ == "__main__":
    unittest.main()
