#!/usr/bin/env python3

import struct
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

SCRIPTS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPTS))

from lz_rviz_viewer.dynamic_occupancy import (
    apply_centered_transform, DynamicOccupancyMap, interpolate_gps, pointcloud2_xyz,
)


def zone(rank=1, x=0.0, y=0.0, z=10.0, radius=2.5, category="safest"):
    return {"rank": rank, "x": x, "y": y, "z": z, "radius": radius, "category": category}


class DynamicOccupancyTests(unittest.TestCase):
    def test_centered_transform_uses_full_elevation_pivot(self):
        angle = np.deg2rad(2.0)
        matrix = np.eye(4)
        matrix[:3, :3] = [
            [np.cos(angle), 0.0, np.sin(angle)],
            [0.0, 1.0, 0.0],
            [-np.sin(angle), 0.0, np.cos(angle)],
        ]
        origin = np.array([100.0, 200.0, 550.0])
        result = apply_centered_transform(origin[None, :], matrix, origin)
        np.testing.assert_allclose(result[0], origin, atol=1e-12)
        wrong = apply_centered_transform(origin[None, :], matrix, [100.0, 200.0, 0.0])
        self.assertGreater(abs(wrong[0, 0] - origin[0]), 19.0)

    def test_interpolates_geographic_uav_at_image_timestamp(self):
        gps = [(1_000_000_000, 10.0, 20.0, 100.0),
               (3_000_000_000, 14.0, 24.0, 104.0)]
        np.testing.assert_allclose(
            interpolate_gps(gps, 2_000_000_000, 45.0), [12.0, 22.0, 57.0]
        )

    def test_decodes_strided_pointcloud2_and_discards_nan(self):
        fields = [
            SimpleNamespace(name="x", offset=0),
            SimpleNamespace(name="y", offset=4),
            SimpleNamespace(name="z", offset=8),
        ]
        data = struct.pack("<ffff", 1.0, 2.0, 3.0, 99.0)
        data += struct.pack("<ffff", np.nan, 5.0, 6.0, 99.0)
        message = SimpleNamespace(fields=fields, width=2, height=1, point_step=16, data=data)
        np.testing.assert_allclose(pointcloud2_xyz(message), [[1.0, 2.0, 3.0]])

    def test_repeated_new_obstacle_blocks_candidate(self):
        occupancy = DynamicOccupancyMap(
            np.empty((0, 3)), [zone()], resolution=0.5,
            obstacle_min_height_m=0.25, min_obstacle_voxels=2,
        )
        obstacle = np.array([[1.0, 0.0, 11.0]])
        occupancy.update(obstacle)
        active, blocked = occupancy.classified_zones()
        self.assertEqual((len(active), len(blocked)), (1, 0))
        occupancy.update(obstacle)
        active, blocked = occupancy.classified_zones()
        self.assertEqual((len(active), len(blocked)), (0, 1))

    def test_margin_obstacle_downgrades_but_keeps_candidate(self):
        occupancy = DynamicOccupancyMap(
            np.empty((0, 3)), [zone()], resolution=0.5, gps_margin_m=2.0,
            obstacle_min_height_m=0.25, min_obstacle_voxels=1,
        )
        occupancy.update(np.array([[3.25, 0.0, 11.0]]))
        active, blocked = occupancy.classified_zones()
        self.assertEqual(len(blocked), 0)
        self.assertEqual(active[0]["category"], "risky")
        self.assertEqual(active[0]["dynamic_state"], "margin")

    def test_baseline_voxel_does_not_create_dynamic_obstacle(self):
        point = np.array([[1.0, 0.0, 11.0]])
        occupancy = DynamicOccupancyMap(
            point, [zone()], resolution=0.5,
            obstacle_min_height_m=0.25, min_obstacle_voxels=1,
        )
        occupancy.update(point)
        active, blocked = occupancy.classified_zones()
        self.assertEqual((len(active), len(blocked)), (1, 0))


if __name__ == "__main__":
    unittest.main()
