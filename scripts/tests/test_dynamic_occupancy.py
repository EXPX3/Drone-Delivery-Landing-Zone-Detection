#!/usr/bin/env python3

import struct
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

SCRIPTS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPTS))

from lz_rviz_viewer.dynamic_occupancy import (
    apply_centered_transform, DynamicOccupancyMap, interpolate_gps, pointcloud2_xyz,
)
from lz_rviz_viewer.validated_pose_replay import (
    timed_xyz, transform_timed_points, ValidatedPoseReplay,
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
        point = np.array([[1.0, 0.0, 10.0]])
        occupancy = DynamicOccupancyMap(
            point, [zone()], resolution=0.5,
            obstacle_min_height_m=0.25, min_obstacle_voxels=1,
        )
        occupancy.update(point)
        active, blocked = occupancy.classified_zones()
        self.assertEqual((len(active), len(blocked)), (1, 0))

    def test_timed_decoder_keeps_point_and_time_masks_paired(self):
        fields = [
            SimpleNamespace(name="x", offset=0, datatype=7),
            SimpleNamespace(name="y", offset=4, datatype=7),
            SimpleNamespace(name="z", offset=8, datatype=7),
            SimpleNamespace(name="time", offset=12, datatype=7),
        ]
        data = struct.pack("<ffff", 1.0, 2.0, 3.0, -0.1)
        data += struct.pack("<ffff", np.nan, 5.0, 6.0, 0.0)
        message = SimpleNamespace(fields=fields, width=2, height=1, point_step=16, data=data)
        points, times = timed_xyz(message)
        np.testing.assert_allclose(points, [[1.0, 2.0, 3.0]])
        np.testing.assert_allclose(times, [-0.1])

    def test_timed_transform_interpolates_scan_translation(self):
        previous, current = np.eye(4), np.eye(4)
        current[0, 3] = 2.0
        result = transform_timed_points(
            np.zeros((3, 3)), np.array([-0.1, -0.05, 0.0]), previous, current,
        )
        np.testing.assert_allclose(result[:, 0], [0.0, 1.0, 2.0])

    def test_validated_pose_replay_enforces_timestamp_contract(self):
        poses = np.repeat(np.eye(4)[None, :, :], 2, axis=0)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "poses.npz"
            np.savez_compressed(path, poses=poses, timestamps=np.array([1.0, 1.1]))
            replay = ValidatedPoseReplay(
                path, {"translation_m": [0, 0, 0], "quaternion_xyzw": [0, 0, 0, 1]},
                maximum_time_error_s=0.02,
            )
            self.assertEqual(replay.pose_index(1_000_000_000)[0], 0)
            with self.assertRaises(RuntimeError):
                replay.pose_index(1_200_000_000)

    def test_adaptive_occupancy_uses_fine_voxels_only_near_zones(self):
        occupancy = DynamicOccupancyMap(
            np.empty((0, 3)), [zone()], resolution=0.25, coarse_resolution=2.0,
            interest_padding_m=0.5, min_obstacle_voxels=1,
        )
        occupancy.update(np.array([[1.0, 0.0, 11.0], [20.0, 20.0, 11.0]]))
        layers = {name: (centers, size) for name, centers, size in occupancy.occupancy_layers(False)}
        self.assertEqual(len(layers["fine"][0]), 1)
        self.assertEqual(layers["fine"][1], 0.25)
        self.assertEqual(len(layers["coarse"][0]), 1)
        self.assertEqual(layers["coarse"][1], 2.0)
        self.assertEqual(len(occupancy.classified_zones()[1]), 1)

    def test_point_cluster_requires_two_of_three_scans(self):
        clustering = {
            "enabled": True, "radius_m": 0.25,
            "minimum_points_near": 5, "minimum_points_mid": 4,
            "minimum_points_far": 3, "near_range_m": 10.0,
            "mid_range_m": 20.0, "far_range_m": 40.0,
            "beyond_far_confirmation_hits": 3,
        }
        occupancy = DynamicOccupancyMap(
            np.empty((0, 3)), [zone()], resolution=0.2, coarse_resolution=1.0,
            obstacle_min_height_m=0.2, vertical_clearance_m=5.0,
            min_obstacle_voxels=2, confirmation_window_scans=3, clustering=clustering,
        )
        cluster = np.array([
            [1.01, 0.01, 10.40], [1.03, 0.02, 10.42], [1.05, 0.03, 10.41],
            [1.07, 0.01, 10.43], [1.09, 0.02, 10.40],
        ])
        occupancy.update(cluster, np.zeros(3))
        occupancy.update(np.empty((0, 3)), np.zeros(3))
        self.assertEqual(len(occupancy.classified_zones()[1]), 0)
        occupancy.update(cluster, np.zeros(3))
        self.assertEqual(len(occupancy.classified_zones()[1]), 1)

    def test_isolated_above_ground_points_do_not_block(self):
        clustering = {
            "enabled": True, "radius_m": 0.25,
            "minimum_points_near": 5, "minimum_points_mid": 4,
            "minimum_points_far": 3, "near_range_m": 10.0,
            "mid_range_m": 20.0, "far_range_m": 40.0,
            "beyond_far_confirmation_hits": 3,
        }
        occupancy = DynamicOccupancyMap(
            np.empty((0, 3)), [zone()], resolution=0.2, coarse_resolution=1.0,
            obstacle_min_height_m=0.2, min_obstacle_voxels=1, clustering=clustering,
        )
        occupancy.update(np.array([[1.0, 0.0, 10.5], [1.8, 0.0, 10.5]]), np.zeros(3))
        self.assertEqual(len(occupancy.classified_zones()[1]), 0)

    def test_temporal_confirmation_tolerates_neighbor_voxel_jitter(self):
        occupancy = DynamicOccupancyMap(
            np.empty((0, 3)), [zone()], resolution=0.2, coarse_resolution=1.0,
            obstacle_min_height_m=0.15, min_obstacle_voxels=2,
            confirmation_window_scans=3,
        )
        occupancy.update(np.array([[1.19, 0.0, 10.39]]))
        occupancy.update(np.array([[1.21, 0.0, 10.41]]))
        self.assertEqual(len(occupancy.classified_zones()[1]), 1)

    def test_confirmed_hazard_can_override_baseline_voxel(self):
        point = np.array([[1.0, 0.0, 10.2]])
        occupancy = DynamicOccupancyMap(
            point, [zone()], resolution=0.2, coarse_resolution=1.0,
            obstacle_min_height_m=0.15, min_obstacle_voxels=1,
        )
        occupancy.update(point)
        self.assertEqual(len(occupancy.classified_zones()[1]), 1)

    def test_rolling_visualization_crops_baseline_horizontally(self):
        baseline = np.array([[0.0, 0.0, 10.0], [50.0, 50.0, 10.0]])
        occupancy = DynamicOccupancyMap(
            baseline, [zone()], resolution=0.2, coarse_resolution=1.0,
        )
        layers = occupancy.occupancy_layers(
            include_baseline=True, center=np.array([0.0, 0.0, 12.0]),
            horizontal_radius_m=10.0,
        )
        self.assertEqual(sum(len(points) for _, points, _ in layers), 1)

    def test_full_map_updates_outside_hazard_processing_radius(self):
        occupancy = DynamicOccupancyMap(
            np.empty((0, 3)), [zone()], resolution=0.2, coarse_resolution=1.0,
            min_obstacle_voxels=1,
        )
        far_point = np.array([[30.0, 30.0, 11.0]])
        occupancy.update(far_point, np.zeros(3), hazard_points=np.empty((0, 3)))
        self.assertEqual(len(occupancy.occupied_centers(include_baseline=False)), 1)
        self.assertEqual(len(occupancy.classified_zones()[1]), 0)


if __name__ == "__main__":
    unittest.main()
