#!/usr/bin/env python3

import sys
import unittest
from pathlib import Path

import numpy as np

SCRIPTS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPTS))

from bag_lio_mapping.build_deskewed_map import interpolate_poses, rotate_vectors
from bag_lio_mapping.compare_maps import column_thickness


class BagLioMappingTests(unittest.TestCase):
    def test_pose_interpolation_translation_and_rotation(self):
        times = np.array([0, 10], dtype=np.int64)
        translations = np.array([[0.0, 0.0, 0.0], [10.0, 0.0, 0.0]])
        quaternions = np.array([[0.0, 0.0, 0.0, 1.0], [0.0, 0.0, 1.0, 0.0]])
        position, rotation, valid = interpolate_poses(
            times, translations, quaternions, np.array([5], dtype=np.int64), 20,
        )
        np.testing.assert_allclose(position, [[5.0, 0.0, 0.0]])
        np.testing.assert_allclose(np.linalg.norm(rotation, axis=1), [1.0])
        np.testing.assert_array_equal(valid, [True])
        rotated = rotate_vectors(rotation, np.array([[1.0, 0.0, 0.0]]))
        np.testing.assert_allclose(rotated, [[0.0, 1.0, 0.0]], atol=1e-12)

    def test_pose_gap_is_rejected(self):
        _, _, valid = interpolate_poses(
            np.array([0, 100], dtype=np.int64), np.zeros((2, 3)),
            np.array([[0.0, 0.0, 0.0, 1.0], [0.0, 0.0, 0.0, 1.0]]),
            np.array([50], dtype=np.int64), 20,
        )
        np.testing.assert_array_equal(valid, [False])

    def test_column_thickness_uses_robust_percentiles(self):
        points = np.array([
            [0.1, 0.1, z] for z in range(10)
        ] + [
            [1.1, 0.1, z] for z in range(4)
        ], dtype=np.float64)
        thickness, cells = column_thickness(points, cell_size=1.0, minimum_points=5)
        self.assertEqual(cells, 1)
        np.testing.assert_allclose(thickness, [8.0])


if __name__ == "__main__":
    unittest.main()
