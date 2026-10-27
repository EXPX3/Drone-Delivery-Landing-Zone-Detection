#!/usr/bin/env python3

import sys
import unittest
from pathlib import Path

import numpy as np

SCRIPTS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPTS))

from semantic_lz.semantic_fusion import SemanticObjectTracker, VisionProposalTracker


class SemanticTrackingTests(unittest.TestCase):
    @staticmethod
    def proposal(x=0.0, confidence=0.8):
        return {
            "class_name": "person", "confidence": confidence,
            "center": np.array([x, 0.0, 1.0]),
            "covariance_xy": np.eye(2) * 0.25,
        }

    def test_camera_person_survives_short_detection_dropout(self):
        tracker = VisionProposalTracker(retention_s={"person": 10.0})
        tracker.update([self.proposal()], 1_000_000_000)
        self.assertEqual(len(tracker.active(10_900_000_000)), 1)
        self.assertEqual(len(tracker.active(11_100_000_000)), 0)

    def test_camera_person_updates_existing_proposal(self):
        tracker = VisionProposalTracker(association_radius_m=2.0)
        tracker.update([self.proposal(0.0)], 1_000_000_000)
        tracker.update([self.proposal(1.0, 0.9)], 2_000_000_000)
        self.assertEqual(len(tracker.proposals), 1)
        self.assertEqual(tracker.proposals[0]["observations"], 2)
        self.assertEqual(tracker.proposals[0]["proposal_id"], 1)

    def test_camera_proposal_with_no_timeout_persists_indefinitely(self):
        tracker = VisionProposalTracker(retention_s={"person": None})
        tracker.update([self.proposal()], 1_000_000_000)
        self.assertEqual(len(tracker.active(3_601_000_000_000)), 1)

    def test_covariance_overlap_merges_duplicate_camera_tracks(self):
        tracker = VisionProposalTracker(
            association_radius_m=0.5, merge_radius_m=2.5,
            mahalanobis_gate=9.21, retention_s={"person": None},
        )
        tracker.update([self.proposal(0.0)], 1_000_000_000)
        tracker.update([self.proposal(1.5)], 2_000_000_000)
        self.assertEqual(len(tracker.proposals), 1)
        self.assertEqual(tracker.proposals[0]["proposal_id"], 1)
        self.assertEqual(tracker.proposals[0]["observations"], 2)
        self.assertGreater(tracker.proposals[0]["covariance_xy"][0, 0], 0.04)

    def test_hard_cap_prevents_broad_covariance_merging_distinct_items(self):
        tracker = VisionProposalTracker(
            association_radius_m=2.0, merge_radius_m=2.5,
            mahalanobis_gate=9.21, retention_s={"person": None},
        )
        broad = self.proposal(0.0)
        broad["covariance_xy"] = np.eye(2) * 100.0
        other = self.proposal(5.9)
        other["covariance_xy"] = np.eye(2) * 100.0
        tracker.update([broad, other], 1_000_000_000)
        self.assertEqual(len(tracker.proposals), 2)

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
