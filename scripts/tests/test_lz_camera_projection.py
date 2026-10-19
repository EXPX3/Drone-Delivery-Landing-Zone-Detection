#!/usr/bin/env python3

import sys
import unittest
from pathlib import Path

import numpy as np
from pyproj import Transformer

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from lz_camera_projection import (  # noqa: E402
    Transform,
    compose,
    fit_planar_rigid,
    latlon_to_utm32,
    project_optical_points,
    scaled_camera_matrix,
    transform_points,
    zone_ring,
)


class ProjectionGeometryTests(unittest.TestCase):
    def test_transform_round_trip(self):
        transform = Transform(np.array([2.0, -3.0, 1.0]), np.array([0.0, 0.0, 0.0, 1.0]))
        identity = compose(transform, transform.inverse())
        np.testing.assert_allclose(identity.matrix, np.eye(4), atol=1e-10)

    def test_transform_points(self):
        transform = Transform(np.array([2.0, 3.0, 4.0]), np.array([0.0, 0.0, 0.0, 1.0]))
        result = transform_points(transform, np.array([[1.0, 1.0, 1.0]]))
        np.testing.assert_allclose(result, [[3.0, 4.0, 5.0]])

    def test_gremsy_optical_axis_maps_to_negative_gimbal_y(self):
        optical_to_gimbal = Transform(
            np.zeros(3), np.array([np.sqrt(0.5), 0.0, 0.0, np.sqrt(0.5)])
        )
        np.testing.assert_allclose(
            optical_to_gimbal.matrix[:3, 2], [0.0, -1.0, 0.0], atol=1e-10
        )

    def test_utm32_matches_proj(self):
        expected = Transformer.from_crs(4326, 25832, always_xy=True).transform(11.6381454, 48.0761299)
        actual = latlon_to_utm32(48.0761299, 11.6381454)
        np.testing.assert_allclose(actual, expected, atol=0.01)

    def test_planar_fit(self):
        source = np.array([[0.0, 0.0], [4.0, 0.0], [0.0, 3.0], [5.0, 2.0]])
        angle = np.deg2rad(23.0)
        rotation = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
        translation = np.array([12.0, -7.0])
        target = (rotation @ source.T).T + translation
        fitted_rotation, fitted_translation, residuals = fit_planar_rigid(source, target)
        np.testing.assert_allclose(fitted_rotation, rotation, atol=1e-10)
        np.testing.assert_allclose(fitted_translation, translation, atol=1e-10)
        self.assertLess(np.max(residuals), 1e-10)

    def test_camera_scaling_and_projection(self):
        matrix = np.array([[1000.0, 0.0, 960.0], [0.0, 1000.0, 540.0], [0.0, 0.0, 1.0]])
        scaled = scaled_camera_matrix(matrix, (1920, 1080), (1280, 720))
        pixels, visible = project_optical_points(np.array([[0.0, 0.0, 10.0]]), scaled, np.zeros(5))
        np.testing.assert_allclose(pixels[0], [640.0, 360.0], atol=1e-8)
        self.assertTrue(visible[0])

    def test_zone_ring_uses_surface_normal(self):
        zone = {
            "x": 10.0, "y": 20.0, "z": 30.0, "radius": 2.5,
            "normal_x": 0.0, "normal_y": np.sqrt(0.5), "normal_z": np.sqrt(0.5),
        }
        ring = zone_ring(zone, vertical_offset_m=4.0, samples=24)
        center = np.array([10.0, 20.0, 34.0])
        offsets = ring - center
        normal = np.array([0.0, np.sqrt(0.5), np.sqrt(0.5)])
        np.testing.assert_allclose(offsets @ normal, 0.0, atol=1e-10)
        np.testing.assert_allclose(np.linalg.norm(offsets, axis=1), 2.5, atol=1e-10)


if __name__ == "__main__":
    unittest.main()
