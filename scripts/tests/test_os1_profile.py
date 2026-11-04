#!/usr/bin/env python3

import unittest
from types import SimpleNamespace

import numpy as np

from lz_rviz_viewer.replay_node import lidar_tf_chain, merge_config
from lz_rviz_viewer.validated_pose_replay import timed_xyz


class Os1ProfileTests(unittest.TestCase):
    def test_profile_merge_preserves_unrelated_configuration(self):
        merged = merge_config(
            {"topics": {"image": "/camera", "lidar_points": "/velodyne_points"}},
            {"topics": {"lidar_points": "/ouster/points"}},
        )
        self.assertEqual(merged["topics"]["image"], "/camera")
        self.assertEqual(merged["topics"]["lidar_points"], "/ouster/points")

    def test_auto_chain_uses_cloud_frame(self):
        message = SimpleNamespace(header=SimpleNamespace(frame_id="/os_sensor"))
        config = {"frames": {"map": "map", "lidar_chain": "auto"}}
        self.assertEqual(lidar_tf_chain(config, message), ["map", "os_sensor"])

    def test_os1_uint32_t_field_is_accepted(self):
        dtype = np.dtype({
            "names": ["x", "y", "z", "t"],
            "formats": ["<f4", "<f4", "<f4", "<u4"],
            "offsets": [0, 4, 8, 12],
            "itemsize": 16,
        })
        values = np.zeros(2, dtype=dtype)
        values["x"] = [1.0, 2.0]
        values["y"] = [3.0, 4.0]
        values["z"] = [5.0, 6.0]
        values["t"] = [100, 200]
        fields = [
            SimpleNamespace(name="x", offset=0, datatype=7),
            SimpleNamespace(name="y", offset=4, datatype=7),
            SimpleNamespace(name="z", offset=8, datatype=7),
            SimpleNamespace(name="t", offset=12, datatype=6),
        ]
        message = SimpleNamespace(
            fields=fields, point_step=16, width=2, height=1,
            data=values.tobytes(), is_bigendian=False,
        )
        points, timestamps = timed_xyz(message)
        np.testing.assert_allclose(points, [[1, 3, 5], [2, 4, 6]])
        np.testing.assert_allclose(timestamps, [100, 200])


if __name__ == "__main__":
    unittest.main()
