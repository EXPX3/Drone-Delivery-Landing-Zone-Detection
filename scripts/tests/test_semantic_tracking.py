#!/usr/bin/env python3

import sys
import unittest
from pathlib import Path

import numpy as np

SCRIPTS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPTS))

from semantic_lz.semantic_fusion import SemanticObjectTracker


class SemanticTrackingTests(unittest.TestCase):
    def test_confirmed_track_absorbs_offset_duplicate(self):
        tracker = SemanticObjectTracker(
            confirmation_frames=3, association_radius_m=0.5,
            duplicate_merge_radius_m=1.5,
            confirmed_duplicate_merge_radius_m=2.5,
        )
        name = np.array(["cargo_box"], dtype=object)
        for frame in range(1, 4):
            tracker.update(np.array([[0.0, 0.0, 1.0]]), name, frame)
        tracker.update(np.array([[2.0, 0.0, 1.0]]), name, 4)
        self.assertEqual(len(tracker.tracks), 1)
        self.assertEqual(tracker.tracks[0].observations, 4)

    def test_distinct_boxes_remain_separate(self):
        tracker = SemanticObjectTracker(
            confirmation_frames=3, association_radius_m=0.5,
            duplicate_merge_radius_m=1.5,
            confirmed_duplicate_merge_radius_m=2.5,
        )
        name = np.array(["cargo_box"], dtype=object)
        for frame in range(1, 4):
            tracker.update(np.array([[0.0, 0.0, 1.0]]), name, frame)
        tracker.update(np.array([[5.9, 0.0, 1.0]]), name, 4)
        self.assertEqual(len(tracker.tracks), 2)


if __name__ == "__main__":
    unittest.main()
