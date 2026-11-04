#!/usr/bin/env python3

import unittest

from lz_rviz_viewer.camera_mode import camera_fusion_available


class CameraModeTests(unittest.TestCase):
    def test_auto_falls_back_to_lidar_only(self):
        topics = {"/ouster/points": "sensor_msgs/msg/PointCloud2"}
        self.assertFalse(camera_fusion_available(topics, "/camera/image", "auto"))

    def test_off_ignores_available_images(self):
        topics = {"/camera/image": "sensor_msgs/msg/Image"}
        self.assertFalse(camera_fusion_available(topics, "/camera/image", "off"))

    def test_auto_enables_available_camera(self):
        topics = {"/camera/image": "sensor_msgs/msg/Image"}
        self.assertTrue(camera_fusion_available(topics, "/camera/image", "auto"))

    def test_required_rejects_missing_camera(self):
        with self.assertRaisesRegex(RuntimeError, "bag lacks"):
            camera_fusion_available({}, "/camera/image", "required")


if __name__ == "__main__":
    unittest.main()
