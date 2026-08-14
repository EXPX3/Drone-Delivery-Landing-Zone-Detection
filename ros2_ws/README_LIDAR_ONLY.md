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
xhost +local:root +local:docker
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
  timestamp_mode:=TIME_FROM_ROS_TIME \
  target_frame:=map \
  use_rviz:=true
```

The retained launch filename preserves compatibility; it contains no Gremsy/RGB components. It starts the Ouster driver, point-cloud/IMU processors, lifecycle detector, automatic configure/activate transitions, and RViz. Use `TIME_FROM_PTP_1588` only when the sensor and TF are on the same synchronized PTP clock. For a direct link-local Ouster connection without PTP, use `TIME_FROM_ROS_TIME`; `TIME_FROM_INTERNAL_OSC` can make the detector reject clouds as stale because the sensor clock is not comparable with ROS wall time.

Tested Jetson direct-link example:

```bash
sudo ./docker/configure_ouster_network.sh
DISPLAY=:1 xhost +local:root +local:docker
docker run -d --name ddlzd-lidar-only --rm \
  --privileged --runtime nvidia --network host --ipc host \
  -e DISPLAY=:1 -e XAUTHORITY= \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  ddlzd-lidar-only:humble \
  ros2 launch ddlzd_ros live_ouster_gremsy.launch.py \
    sensor_hostname:=169.254.210.24 \
    lidar_mode:=1024x10 \
    timestamp_mode:=TIME_FROM_ROS_TIME \
    target_frame:=os_sensor \
    use_rviz:=true
```

Stop the launched demo completely:

```bash
docker rm -f ddlzd-lidar-only
```

## Current Ouster sensor settings

- Sensor: `OS-1-128`, SN `122608004931` - 128-channel Ouster lidar currently running.
- IP: `169.254.210.24` - sensor address on the link-local lidar network.
- UDP destination: `169.254.50.1` - Jetson interface receiving lidar packets.
- Lidar mode: `1024x10` - 1024 columns per scan at 10 Hz.
- Lidar UDP profile: `RNG19_RFL8_SIG16_NIR16` - range, reflectivity, signal, and near-IR fields are streamed.
- IMU UDP profile: `LEGACY` - standard Ouster IMU packet format.
- Lidar UDP port: `41229` - current auto-assigned lidar data port.
- IMU UDP port: `45166` - current auto-assigned IMU data port.
- Columns per packet: `16` - each UDP packet carries 16 lidar columns.
- Return order: `STRONGEST_TO_WEAKEST` - strongest return is prioritized first.
- Timestamp mode on sensor: `TIME_FROM_INTERNAL_OSC` - sensor hardware config uses its internal oscillator.
- Launch timestamp mode used for working demo: `TIME_FROM_ROS_TIME` - ROS driver stamped messages with ROS time to avoid stale-cloud rejection.
- Operating mode: `NORMAL` - sensor is actively scanning.
- Azimuth window: `0-360 deg` - full 360-degree scan enabled.
- Minimum range threshold: `50 cm` - points closer than 0.5 m are filtered by sensor setting.
- Signal multiplier: `1` - default signal scaling.
- Phase lock: `disabled` - no phase locking active.
- Sync/NMEA input: active-high, NMEA `9600 baud` - external timing inputs are configured but not used for the working demo.

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
