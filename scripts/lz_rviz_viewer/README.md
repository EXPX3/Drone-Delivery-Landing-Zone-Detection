# Fast RViz Landing-Zone Replay

For the complete current-state runbook, validation results, and troubleshooting, see
[`../DEMO_rviz_3dlz_to_lz_in_image.md`](../DEMO_rviz_3dlz_to_lz_in_image.md).

This viewer is the efficient native ROS 2 alternative to the synchronized browser
viewer. It publishes a downsampled static `PointCloud2`, batched landing-zone
`MarkerArray` messages, live camera/frustum markers, and the annotated camera image.
All live output for a replay step uses the same ROS image timestamp.
RViz renders the 3D scene, while the native ROS `image_view` process displays the
camera topic in a separate lightweight window.

Launch with the existing `cargopack:2026` image:

```bash
cd /home/gvb/Documents/robotspace/DDLZD_ws/Drone-Delivery-Landing-Zone-Detection
scripts/lz_rviz_viewer/run_rviz_replay.sh
```

Optional positional arguments are bag path, playback rate, and visualization FPS:

```bash
scripts/lz_rviz_viewer/run_rviz_replay.sh \
  /media/gvb/ssd24ubuntu/bags/Bagfilles_Feb13_2026/recovered_bag 1.0 12
```

Topics:

- `/ddlzd/pointcloud`: static local-origin point cloud;
- `/ddlzd/landing_zones`: static risk-colored solutions;
- `/ddlzd/replay_markers`: UAV, trajectory, frustum, footprint, and pink visible zones;
- `/ddlzd/camera_overlay`: synchronized annotated camera image, resized to
  `640x360` by default to keep DDS and rendering overhead low.

The point cloud is limited to 250,000 deterministic samples by default and shifted
to a local XY origin to avoid float32 precision loss in RViz. The camera image carries
the replay frame, bag-relative time, and original ROS timestamp.
The launcher enables NVIDIA PRIME OpenGL offload for responsive orbit and wheel zoom
on the hybrid RTX laptop. The LiDAR map is static by design; the UAV, trajectory,
camera frustum, footprint, and pink visible-zone layer update during replay.

The first launch still scans the bag to reconstruct TF/GNSS registration. Once replay
starts, rendering is native RViz/Ogre and avoids Plotly, HTTP polling, JPEG encoding,
and browser-side point-cloud redraws.

Display a standalone local-frame PCD with the mapping-audit RViz configuration:

```bash
scripts/lz_rviz_viewer/run_mapping_audit_viewer.sh \
  pcds/bag_only_deskewed_map.pcd
```

Standalone mode uses local `(0, 0)` as the 100 m ROI center, derives the RViz
floating-point origin from the cloud, and deterministically displays at most one
million points. Calling the launcher without an argument retains the original
four-layer Bayern-assisted audit view and adds the georeferenced Bayern DOP20
orthophoto as a ground surface. The primary independently toggleable displays are:

- `Bayern DOP20 Orthophoto`;
- `Bayern Static Map PCD`, loaded from
  `pcds/696_5328_cropped_by_bayernatlas_20260812084517.pcd`;
- `Full Corrected LiDAR Scene (ground + obstacles)`, loaded from
  `pcds/lidar_mapping_accuracy/direct_corrected_pointcloud.pcd`.

Use their checkboxes in RViz's Displays panel to show any layer alone or all layers
together. The two PCDs are already in the same georeferenced coordinate frame. The
viewer applies one shared local rendering origin to all three layers solely to avoid
float32 precision loss at UTM coordinates.

The full corrected scene is the appropriate layer when every corrected return must
remain available: its per-scan ground-derived vertical correction is applied to the
entire LiDAR scan before writing, so ground, people, cargo boxes, vehicles, trees,
and other elevated returns remain in one map. `Corrected Update-Only Ground` and
`Confirmed Scene Changes` are filtered diagnostic subsets and must not be treated as
a complete scene replacement.

The primary cloud uses a fixed Z-axis rainbow scale from `-0.5 m` to `2.2 m`
relative to the displayed cloud origin. This gives near-ground boxes and human-height
returns more color separation than automatic scaling over trees and other tall data.
