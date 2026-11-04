# OS1 Real-UAV Readiness

## Scope

`TAG: os1` adapts recorded-bag replay to Ouster ROS 2 conventions:

- point topic `/ouster/points`;
- automatic use of `PointCloud2.header.frame_id` (`os_sensor` or `os_lidar`);
- OS1 relative firing-time field `t` in addition to `time`;
- maximum input range `120 m`;
- the Velodyne bag's optimized poses and extrinsic are disabled.

This does **not** make `replay_node.py` a live flight node. It still opens a completed
ROS 2 bag and fits geographic registration from recorded TF/GPS. Do not use it for
real-time landing decisions on the aircraft.

## Required Real-UAV Inputs

Before recording or implementing direct live operation, provide:

- `/ouster/points` with `x`, `y`, `z`, and `t` (or `time`) fields;
- a measured static transform from `base_link` to the cloud frame;
- synchronized `/tf`, `/tf_static`, GNSS, and camera image topics;
- measured OS1-to-body and camera-to-body extrinsics;
- camera intrinsics for the deployed resolution;
- a common clock source, preferably PTP/GPS, for OS1, camera, and flight computer.

Run the preflight checker on the UAV computer while all sensors are publishing:

```bash
source /opt/ros/humble/setup.bash
python3 scripts/semantic_lz/os1_preflight.py \
  --points-topic /ouster/points \
  --image-topic /GremsyVIO/image/cpu \
  --gps-topic /mavros/global_position/global \
  --base-frame base_link \
  --duration 15 \
  --output /tmp/ddlzd_os1_preflight.json
```

The command returns `0` only when point fields, timestamps, TF, camera, and finite GPS
are all observed. Topic and frame arguments must be changed to the deployed names.

## OS1 Bag Replay

After recording a bag with the required topics, replay the OS1 profile with the sixth
launcher argument:

```bash
scripts/lz_rviz_viewer/run_rviz_replay.sh \
  /path/to/os1_bag 1.0 12 camera_lidar_correlated_only on os1 auto
```

For an OS1 recording without camera images, use `off` as the seventh argument. In
`auto`, a missing image topic automatically selects LiDAR-only mapping and landing-zone
detection. Camera availability never gates geometric occupancy or LZ safety updates.

This profile intentionally uses bag TF rather than the old Velodyne optimized-pose
archive. A new OS1-specific trajectory and calibration must be generated before
validated deskewed replay can be enabled.
