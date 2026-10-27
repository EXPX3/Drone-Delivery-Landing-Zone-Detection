import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from semantic_lz.ground_reference import local_detection_ground_estimate


def test_local_plane_ignores_elevated_object_points():
    rng = np.random.default_rng(4)
    xy = rng.uniform(-5.0, 5.0, size=(800, 2))
    ground_z = 12.0 + 0.02 * xy[:, 0] - 0.01 * xy[:, 1]
    ground = np.column_stack((xy, ground_z + rng.normal(0.0, 0.015, len(xy))))
    object_xy = rng.normal(0.0, 0.15, size=(35, 2))
    objects = np.column_stack((object_xy, np.full(len(object_xy), 13.2)))
    result = local_detection_ground_estimate(objects, np.vstack((ground, objects)))

    assert result.diagnostics["fit_success"]
    expected = 12.0 + 0.02 * object_xy[:, 0] - 0.01 * object_xy[:, 1]
    np.testing.assert_allclose(result.ground_z, expected, atol=0.03)
    assert result.diagnostics["slope_deg"] < 2.0


def test_local_plane_fails_without_enough_support():
    detection = np.array([[0.0, 0.0, 1.0]])
    result = local_detection_ground_estimate(detection, detection, minimum_inliers=20)
    assert not result.diagnostics["fit_success"]
    assert np.isnan(result.ground_z).all()
