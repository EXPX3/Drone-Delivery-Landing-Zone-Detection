# Live Ouster LiDAR-only landing-zone demo

This branch runs the complete landing-zone pipeline from an Ouster `PointCloud2` stream. It has no camera, OpenCV, or `cv_bridge` dependency. Every candidate is an exact 2.5 m-radius horizontal footprint and is classified from point-cloud geometry:

- `SAFEST` (green): risk score `<= 0.33` after temporal stabilization.
- `SAFE` (orange): risk score `<= 0.62` after temporal stabilization.
- `RISKY` (red): score above `0.62`, or a hard geometry rejection.
- `UNKNOWN` (gray): insufficient observed clearance or too few stable observations.

The geometry score combines obstacle count (27%), clearance (23%), relief (17%), roughness (13%), slope (12%), and footprint coverage (8%). All detector, risk, timing, and tracking parameters are centralized in `src/ddlzd_ros/config/live_fusion.yaml`.

## Requirements

- Jetson/aarch64 compatible with the NVIDIA Isaac ROS Humble base image.
- Docker, NVIDIA Container Runtime, and X11 access for RViz.
- Reachable Ouster OS1.
- Gravity-aligned `map`/`odom` target frame and timestamped Ouster TF.
- With `input_is_motion_compensated:=false`, Ouster's per-point `t` field.

## Build and run Docker

From the repository root:

```bash
docker build --file docker/Dockerfile.live_demo_cargopack --tag ddlzd-lidar-only:humble .
sudo ./docker/configure_ouster_network.sh
xhost +local:docker
docker run --rm -it \
  --name ddlzd-lidar-only \
  --privileged --gpus all --network host --ipc host \
  -e DISPLAY -e XAUTHORITY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  ddlzd-lidar-only:humble
```

The entrypoint sources ROS 2 Humble and `/ros2_ws/install/setup.bash`.

## One-command Ouster + detector + RViz launch

Inside the container:

```bash
ros2 launch ddlzd_ros live_ouster_gremsy.launch.py \
  sensor_hostname:=192.168.1.10 \
  lidar_mode:=1024x10 \
  timestamp_mode:=TIME_FROM_PTP_1588 \
  target_frame:=map \
  use_rviz:=true
```

The retained launch filename preserves compatibility; it contains no Gremsy/RGB components. It starts the Ouster driver, point-cloud/IMU processors, lifecycle detector, automatic configure/activate transitions, and RViz. Use `TIME_FROM_INTERNAL_OSC` if PTP is not configured, with TF on the same clock.

## Use an existing live cloud or bag

```bash
ros2 launch ddlzd_ros live_fusion_only.launch.py \
  point_cloud_topic:=/ouster/points \
  target_frame:=map \
  input_is_motion_compensated:=false \
  use_rviz:=true
```

Set `input_is_motion_compensated:=true` only for an already-deskewed cloud. Override the central configuration with `config_file:=/config/live_fusion.yaml`.

## Outputs and checks

RViz shows the registered cloud and live colored 2.5 m candidate circles. Topics:

- `/landing_zone/live_landing_zone/local_cloud`
- `/landing_zone/live_landing_zone/markers`
- `/landing_zone/live_landing_zone/zones`
- `/diagnostics`

```bash
ros2 lifecycle get /landing_zone/live_landing_zone
ros2 topic hz /ouster/points
ros2 topic hz /landing_zone/live_landing_zone/local_cloud
ros2 topic echo /landing_zone/live_landing_zone/zones --once
ros2 topic echo /diagnostics --once
ros2 run tf2_ros tf2_echo map os_sensor
```

Missing TF, stale clouds, incomplete clearance coverage, and invalid parameters fail closed rather than producing favorable zones.

## Build and test the ROS workspace

From `ros2_ws` in ROS 2 Humble:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```
