# Landing-Zone Camera Demonstration

## Current state

The recommended demonstration is the native ROS 2/RViz implementation in
`scripts/lz_rviz_viewer`. It replays the recorded camera and UAV motion against the
static Bayern LiDAR point cloud and risk-ranked landing-zone solutions.

Current validated behavior:

- Branch: `lz-cam-viewer`
- ROS environment: local Docker image `cargopack:2026`, ROS 2 Humble
- Point cloud: 250,000 deterministic samples in RViz, shifted to a local XY origin
- Landing zones: 1,356 static risk-ranked candidates
- Replay: 12 visualization frames per second at 1x bag time
- Camera and 3D markers: identical ROS image timestamps for each replay step
- Horizontal GNSS/map registration RMSE: approximately 0.077 m
- Browser cross-check: 2,543 frames checked with zero image/frustum set mismatches
- Hardware rendering: NVIDIA PRIME offload, OpenGL 4.6

The LiDAR map and all candidate landing zones are static. The UAV, trajectory,
camera frustum, ground footprint, and light-pink camera-visible zone subset move as
the bag advances. Camera overlays retain the risk colors used by the offline viewer:

- Light green: top 30 candidates
- Green: safest
- Orange: safe
- Red: risky
- Light pink: zones currently selected by the camera frustum in the 3D view

## Inputs

Default files and topics:

- Bag: `/media/gvb/ssd24ubuntu/bags/Bagfilles_Feb13_2026/recovered_bag`
- Point cloud: `pcds/696_5328_cropped_by_bayernatlas_20260812084517.pcd`
- Ranked zones: `pcds/region_growing_landing_zones_max10000_radius2_5_to3_risk_ranked.csv`
- Camera topic: `/GremsyVIO/image/cpu`
- GNSS topic: `/mavros/global_position/global`
- Pose topics: `/tf` and `/tf_static`
- Projection config: `scripts/config/lz_camera_projection.yaml`

## Start the recommended demonstration

From the repository root:

```bash
cd /home/gvb/Documents/robotspace/DDLZD_ws/Drone-Delivery-Landing-Zone-Detection
scripts/lz_rviz_viewer/run_rviz_replay.sh
```

The launcher opens two windows:

1. RViz2: point cloud, all risk-ranked zones, UAV trajectory, camera footprint,
   frustum, and dynamic light-pink visible-zone circles.
2. ROS `image_view`: annotated `640x360` camera frames with replay frame number,
   relative bag time, ROS timestamp, and projected landing-zone circles.

The canonical RViz configuration is copied to `/tmp` inside the container before
launch. Interactive display toggles and camera changes therefore apply to that run
without modifying or disabling the tracked demonstration configuration.

The point cloud appears first. Dynamic replay begins after the full TF/GNSS scan,
normally about 35-40 seconds on the current machine.

Optional arguments are bag path, playback rate, and visualization FPS:

```bash
scripts/lz_rviz_viewer/run_rviz_replay.sh \
  /media/gvb/ssd24ubuntu/bags/Bagfilles_Feb13_2026/recovered_bag 1.0 12
```

For a lighter run:

```bash
scripts/lz_rviz_viewer/run_rviz_replay.sh \
  /media/gvb/ssd24ubuntu/bags/Bagfilles_Feb13_2026/recovered_bag 1.0 8
```

RViz navigation:

- Mouse wheel: zoom
- Left drag: orbit
- Middle drag: pan
- `Move Camera` toolbar button: restore camera navigation
- `F`: focus the selected point or marker

Stop the demonstration by closing RViz or running:

```bash
docker stop ddlzd-lz-rviz
```

## ROS topics

The RViz replay node publishes:

```text
/ddlzd/pointcloud       sensor_msgs/msg/PointCloud2
/ddlzd/landing_zones    visualization_msgs/msg/MarkerArray
/ddlzd/replay_markers   visualization_msgs/msg/MarkerArray
/ddlzd/camera_overlay   sensor_msgs/msg/Image
```

Inspect the running system from the host:

```bash
docker logs -f ddlzd-lz-rviz
```

Inspect topic connections inside the container:

```bash
docker exec ddlzd-lz-rviz bash -lc '
  source /opt/ros/humble/setup.bash
  export ROS2CLI_NO_DAEMON=1
  ros2 topic list | grep ddlzd
  ros2 node info /rviz
'
```

Each of the four DDLZD topics should report an active subscriber while the demo is
running. The replay log reports frame milestones such as:

```text
Published frame 120 at bag t=11.99s
```

## Browser audit mode

The browser implementation is slower but provides the independent image/frustum
consistency audit and ID lists. Start it only after stopping any previous instance:

```bash
cd /home/gvb/Documents/robotspace/DDLZD_ws/Drone-Delivery-Landing-Zone-Detection
scripts/lz_bag_replay_viewer/run_replay.sh
```

Then open:

```bash
google-chrome http://127.0.0.1:8765/
```

The browser requests the JPEG associated with the exact state sequence, so the
displayed camera frame and Plotly frustum cannot come from different replay steps.
Use browser mode for validation and RViz mode for responsive demonstration.

## Verification

Run the geometry and rendering tests:

```bash
/home/gvb/Documents/robotspace/venvs/praktikum_flightplanning_venv/bin/python \
  -m unittest discover -s scripts/tests -v
```

Expected result: 10 tests pass.

Validate shell and Python syntax:

```bash
bash -n scripts/lz_rviz_viewer/run_rviz_replay.sh
python3 -m py_compile \
  scripts/lz_rviz_viewer/replay_node.py \
  scripts/lz_bag_replay_viewer/replay_server.py
```

## Troubleshooting

### RViz opens but displays nothing

Wait for `Published 250000 cloud points and 1356 landing zones` in `docker logs`.
Confirm that RViz subscribes to `/ddlzd/pointcloud`, `/ddlzd/landing_zones`, and
`/ddlzd/replay_markers` using the topic inspection command above.

### RViz cannot orbit or zoom smoothly

The launcher requires the NVIDIA container runtime and passes PRIME offload settings.
The RViz log should report OpenGL 4.6 without `Failed to query drm device`. Confirm:

```bash
docker logs ddlzd-lz-rviz 2>&1 | grep -E 'OpenGl|drm|libGL'
```

### Camera is initially blank

The camera starts only after registration completes. Wait for `Registration ready`
and `Published frame 1` in the logs. The point cloud is available before that scan
finishes.

### Existing container name prevents launch

```bash
docker stop ddlzd-lz-rviz
scripts/lz_rviz_viewer/run_rviz_replay.sh
```

## Calibration limitation

The projection configuration is still marked `provisional`. Camera intrinsics are
scaled from the known 1920x1080 matrix, the rigid nadir optical rotation is modeled,
and the vertical datum correction is currently 45 m. This demonstration is suitable
for algorithm development and visual validation, but not operational flight decisions
until camera calibration, lever arm, and vertical datum are surveyed and the config
status is changed to `measured`.
