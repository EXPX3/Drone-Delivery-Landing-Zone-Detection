import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from semantic_lz.semantic_fusion import (
    SemanticObjectTracker, SemanticResult, SemanticVoxelMemory,
    canonical_class, compact_supported_clusters, filter_semantic_result,
    labels_at_pixels,
)


class SemanticFusionTests(unittest.TestCase):
    def test_canonical_classes_cover_safety_obstacles(self):
        self.assertEqual(canonical_class("person"), "person")
        self.assertEqual(canonical_class("cardboard box"), "cargo_box")
        self.assertEqual(canonical_class("white rectangular obstacle"), "cargo_box")
        self.assertEqual(canonical_class("black rectangular obstacle"), "cargo_box")
        self.assertEqual(canonical_class("car"), "vehicle")

    def test_projected_pixels_receive_mask_label(self):
        mask = np.zeros((1, 8, 8), dtype=bool)
        mask[0, 2:6, 2:6] = True
        result = SemanticResult(["person"], np.array([0.9]), mask)
        pixels = np.array([[3.0, 4.0], [7.0, 7.0], [3.0, 4.0]])
        valid = np.array([True, True, False])
        self.assertEqual(labels_at_pixels(result, pixels, valid).tolist(), [1, 0, 0])

    def test_image_filter_rejects_low_confidence_and_oversized_masks(self):
        masks = np.zeros((3, 10, 10), dtype=bool)
        masks[0, :2, :2] = True
        masks[1, :2, :2] = True
        masks[2, :, :] = True
        result = SemanticResult(
            ["cargo_box", "cargo_box", "cargo_box"],
            np.array([0.8, 0.05, 0.8]), masks,
        )
        filtered = filter_semantic_result(
            result, {"cargo_box": 0.1}, {"cargo_box": 0.08},
        )
        self.assertEqual(filtered.class_names, ["cargo_box"])

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

    def test_compact_support_rejects_broad_mask_leakage(self):
        compact = np.array([[0.0, 0.0, 0.5], [0.1, 0.0, 0.6], [0.2, 0.0, 0.5]])
        broad = np.array([[2.0 + index * 0.4, 0.0, 0.5] for index in range(8)])
        accepted = compact_supported_clusters(
            np.vstack((compact, broad)), "cargo_box", cluster_radius_m=0.5,
            minimum_points=2, maximum_radial_p95_m=0.4,
        )
        self.assertEqual(len(accepted), 1)
        self.assertEqual(len(accepted[0]), 3)

    def test_tracker_expires_stale_object_and_keeps_boxes_bright_yellow(self):
        tracker = SemanticObjectTracker(
            confirmation_frames=1, maximum_age_frames=2, association_radius_m=0.5,
        )
        tracker.update(np.array([[0.0, 0.0, 0.5]]), np.array(["cargo_box"]), frame_index=1)
        tracker.update(np.array([[2.0, 0.0, 0.5]]), np.array(["cargo_box"]), frame_index=2)
        _, colors = tracker.point_cloud()
        self.assertTrue(np.all(colors == np.array([255, 255, 0])))
        tracker.expire(4)
        self.assertEqual([track.track_id for track in tracker.tracks], [2])

    def test_only_confirmed_tracks_consume_display_ids(self):
        tracker = SemanticObjectTracker(
            confirmation_frames=2, association_radius_m=0.25, maximum_age_frames=0,
        )
        cargo = np.array(["cargo_box"], dtype=object)
        tracker.update(np.array([[0.0, 0.0, 0.5]]), cargo, frame_index=1)
        tracker.update(np.array([[2.0, 0.0, 0.5]]), cargo, frame_index=2)
        tracker.update(np.array([[2.0, 0.0, 0.5]]), cargo, frame_index=3)
        tracker.update(np.array([[4.0, 0.0, 0.5]]), cargo, frame_index=4)
        tracker.update(np.array([[4.0, 0.0, 0.5]]), cargo, frame_index=5)
        self.assertEqual([track.track_id for track in tracker.confirmed_tracks()], [2, 3])
        self.assertEqual([track.display_id for track in tracker.confirmed_tracks()], [1, 2])
        self.assertIsNone(tracker.tracks[0].display_id)


if __name__ == "__main__":
    unittest.main()
