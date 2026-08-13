# Live exact-2.5 m landing-zone detection

This workspace implements live ROS 2 Humble landing-zone detection for an Ouster OS1 and a calibrated RGB camera carried by a Gremsy gimbal. It is the live counterpart of the `demo_cargopack` offline experiment and enforces a circle radius of exactly 2.5 m.

The implementation is intended for controlled academic research. It is not a certified flight-safety component and must not directly authorize an autonomous landing without an independently validated supervisory controller.

## Packages

- `ddlzd_core`: PCL geometry, hard rejection gates, fixed-scale risk fusion, and unit tests.
- `ddlzd_msgs`: complete landing-zone metrics and category interfaces.
- `ddlzd_ros`: lifecycle node, Ouster deskew, rolling map, RGB projection, temporal tracking, RViz markers, diagnostics, and launch files.

No scan is serialized to PCD in the live data path. A bounded rolling cloud is held in memory and the newest snapshot replaces any pending snapshot if processing falls behind.

## Data contract

The complete fusion result is valid only when all of the following are available:

1. An Ouster `sensor_msgs/PointCloud2` with `x`, `y`, `z`, `intensity`, and `t` fields. The official `original` Ouster point type satisfies this contract; `t` is the per-point nanosecond offset from frame start.
2. A rectified `bgr8` or convertible RGB camera image.
3. A calibrated `sensor_msgs/CameraInfo` whose dimensions and optical frame match the image.
4. Timestamped TF from the LiDAR frame to a gravity-aligned local frame for the full LiDAR scan duration.
5. Timestamped TF from that local frame to the moving camera optical frame at the image timestamp. This transform must include the Gremsy gimbal encoder attitude and the calibrated rigid mount transforms.
6. A common or measured time base for Ouster, camera, vehicle state, and gimbal attitude.

The node does not invent an extrinsic transform, a camera model, a gimbal angle, or a missing timestamp. A missing transform or invalid calibration rejects that measurement. Missing or insufficient RGB evidence produces `UNKNOWN`, never `SAFEST`.

If an upstream component already publishes a correctly motion-compensated cloud, the fusion-only launch accepts it when `input_is_motion_compensated:=true`. That declaration is an explicit data contract. When false, the node requires Ouster's `t` field, rejects scans whose per-point offsets exceed `max_scan_duration_sec`, and deskews each point using interpolated SE(3) transforms. The deskewed scan's midpoint is used as its synchronization and output timestamp, minimizing the maximum camera-to-LiDAR time separation across the scan.

## Detection method

For each point-cloud snapshot, `ddlzd_core` performs the following operations:

1. Reject non-finite points and voxel-filter the rolling cloud.
2. Estimate local normals once and form smooth connected surface regions using PCL region growing.
3. Fit one gravity-constrained RANSAC plane to each region and reject regions without enough inliers.
4. Evaluate a regular world-frame lattice over the accepted region bounds; duplicate lattice cells are evaluated once.
5. Query each exact 2.5 m footprint plus its safety margin in horizontal metric space.
6. Orient the regional normal toward gravity-positive Z and calculate slope.
7. Calculate footprint inlier density, RMS point-to-plane roughness, robust 5–95% relief, and angular/radial observation coverage.
8. Search the footprint plus measured safety margin for the hard obstacle count, and search to 12 m for nearest observed obstacle clearance. Clearance-area observation coverage is reported separately and insufficient clearance coverage makes the score unobservable.
9. Apply hard geometry gates before risk scoring, then apply non-maximum suppression and cap the published candidate count.

The radius is checked in the core constructor and cannot be changed from 2.5 m in this branch. As in the offline geospatial buffer, radius is defined in the gravity-aligned horizontal XY plane; the displayed boundary is lifted onto the fitted local plane without changing its XY radius. The checked-in exact-radius configuration retains the base experiment's zero expansion. Any operational expansion must come from a measured navigation, wind, and control-error budget rather than an invented constant.

### RGB evidence

The offline orthophoto workflow cannot use its affine world-to-pixel mapping for a perspective gimbal camera. The live implementation instead projects accumulated 3D points with the calibrated pinhole model:

\[
\begin{bmatrix}u\\v\\1\end{bmatrix}
\sim P\,T_{camera\leftarrow local}(t_{image})
\begin{bmatrix}x\\y\\z\\1\end{bmatrix},
\]

where `P` is the rectified projection matrix from `CameraInfo` (not the raw-image intrinsic matrix `K`).

A LiDAR depth buffer removes occluded projected points. RGB features retain the explainable structure of the current offline implementation: excess green, VARI, HSV saturation, local intensity texture, darkness, and brightness. Unlike the offline tile script, live thresholds use fixed configured physical/image scales rather than per-frame percentiles. This prevents an image containing no vegetation from being forced to contain a fixed percentage of vegetation.

Camera coverage is measured in angular/radial footprint bins, not raw point count, to reduce sampling-density bias. Tree, grass, and texture statistics are computed only from visible projected LiDAR returns.

### Risk and categories

Hard geometry rejection always has precedence over RGB. RGB cannot convert a geometrically rejected footprint into a valid landing zone.

For geometry-valid candidates, the implemented score is

\[
R=0.23R_o+0.18R_c+0.17R_v+0.14R_r+0.10R_q+0.07R_t+0.06R_s+0.05R_{cov}-0.10B_g,
\]

where the terms are obstacle count, obstacle clearance, tree/vegetation evidence, relief, roughness, texture, slope, observation deficit, and clear grass-like surface evidence. The weights, category boundaries, and the obstacle/clearance/relief/roughness/slope normalization ranges are taken directly from `scripts/rank_landing_zone_risk.py` on `demo_cargopack`. The live Ouster stream has no LAS class-20 field, so the offline class-20 term is explicitly replaced by a LiDAR clearance-area observation-deficit term of the same weight. This substitution is a stated model change and requires ablation and recalibration; it is not presented as equivalent evidence. Normalization uses fixed configured ranges and is never relative to other candidates in the current frame.

The default research categories reproduce the offline score boundaries:

- `SAFEST`: score no greater than 0.33, complete geometry and camera evidence, and temporal stability.
- `SAFE`: score no greater than 0.62 with the same evidence requirements.
- `RISKY`: higher score or any hard geometry rejection.
- `UNKNOWN`: insufficient footprint/clearance observation, insufficient camera coverage, missing RGB/TF/calibration, or insufficient temporal observations.

The `risk_score` field is IEEE NaN when the score is not observable because camera evidence is unavailable or below the configured coverage threshold. This prevents a missing modality from silently appearing as a favorable numeric score.

These numeric boundaries are hypotheses inherited from the offline study. They must be calibrated and reported against annotated live data before scientific or operational use.

## Build

On ROS 2 Humble:

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --event-handlers console_direct+
colcon test-result --verbose
source install/setup.bash
```

The dedicated Jetson image uses the same base image as the target system:

```bash
docker build --network=host \
  -f docker/Dockerfile.live_demo_cargopack \
  -t ddlzd-live:humble .
```

Run `docker/configure_ouster_network.sh` as root on the Jetson host at boot. Run the container with host networking so Ouster UDP traffic and CycloneDDS discovery are available.

The complete reproducible host/container sequence is:

```bash
git clone --branch live_demo_cargopack --single-branch \
  https://github.com/EXPX3/Drone-Delivery-Landing-Zone-Detection.git
cd Drone-Delivery-Landing-Zone-Detection

sudo docker/configure_ouster_network.sh
docker build --network=host \
  --file docker/Dockerfile.live_demo_cargopack \
  --tag ddlzd-live:humble .

xhost +si:localuser:root
docker run --rm --interactive --tty \
  --network host \
  --runtime nvidia \
  --env DISPLAY \
  --env QT_X11_NO_MITSHM=1 \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --volume /etc/localtime:/etc/localtime:ro \
  --name ddlzd-live \
  ddlzd-live:humble
```

`--runtime nvidia` is required on JetPack Docker installations that use the NVIDIA runtime. If the target uses the NVIDIA Container Toolkit CDI interface instead, use that deployment's documented GPU flag. Do not run the detector with both forms simultaneously.

The image entrypoint sources ROS 2 Humble and `/ros2_ws/install/setup.bash`. Commands below are therefore entered directly in the container shell.

## One configuration file

All non-hardware parameters are in:

```text
/ros2_ws/install/share/ddlzd_ros/config/live_fusion.yaml
```

The source file is `ros2_ws/src/ddlzd_ros/config/live_fusion.yaml`. It contains the selected `region_growing_exact_radius` algorithm, exact-radius detector thresholds, rolling-window and deskew timing, semantic evidence, risk normalizers, temporal tracking, and observation-coverage requirements. Unsupported algorithm names fail configuration instead of silently selecting another implementation.

Hardware bindings remain required launch arguments because the repository cannot know the deployed OS1 address, time source, Gremsy topics, or gravity-aligned frame. To use a modified parameter file mounted at runtime:

```bash
--volume /absolute/path/live_fusion.yaml:/config/live_fusion.yaml:ro
```

and pass `config_file:=/config/live_fusion.yaml` to the launch command.

## Launch

The integrated launch has no hardware defaults. Each required launch argument must be the real value from the deployed system:

```bash
ros2 launch ddlzd_ros live_ouster_gremsy.launch.py \
  sensor_hostname:="${OS1_HOSTNAME:?set OS1_HOSTNAME}" \
  lidar_mode:="${OS1_LIDAR_MODE:?set OS1_LIDAR_MODE}" \
  timestamp_mode:="${OS1_TIMESTAMP_MODE:?set OS1_TIMESTAMP_MODE}" \
  image_topic:="${GREMSY_RECTIFIED_IMAGE_TOPIC:?set GREMSY_RECTIFIED_IMAGE_TOPIC}" \
  camera_info_topic:="${GREMSY_CAMERA_INFO_TOPIC:?set GREMSY_CAMERA_INFO_TOPIC}" \
  target_frame:="${GRAVITY_ALIGNED_LOCAL_FRAME:?set GRAVITY_ALIGNED_LOCAL_FRAME}" \
  config_file:=/ros2_ws/install/share/ddlzd_ros/config/live_fusion.yaml \
  use_rviz:=true
```

This single launch command starts the official Ouster driver, its point-cloud and IMU processors, the lifecycle landing-zone node, automatic configure/activate transitions, and RViz. It configures Ouster `point_type=original` so the live node receives the per-point `t` field needed for deskew, disables organized output, subscribes the detector to `/ouster/points`, and enables driver reconnection. The Gremsy camera publisher and calibrated/gimbal TF publisher must already be running because their vendor-specific node and interfaces are not part of this repository.

The integrated live path is therefore:

```text
OS1 UDP -> ouster_ros -> /ouster/points -> deskew + rolling map -> exact 2.5 m geometry
Gremsy rectified image + CameraInfo + timestamped gimbal TF -> RGB projection -> risk fusion
fusion -> LandingZoneArray + live cloud + colored RViz MarkerArray + annotated image
```

For an existing driver and processing graph:

```bash
ros2 launch ddlzd_ros live_fusion_only.launch.py \
  point_cloud_topic:="${DESKEW_INPUT_TOPIC:?set DESKEW_INPUT_TOPIC}" \
  image_topic:="${GREMSY_RECTIFIED_IMAGE_TOPIC:?set GREMSY_RECTIFIED_IMAGE_TOPIC}" \
  camera_info_topic:="${GREMSY_CAMERA_INFO_TOPIC:?set GREMSY_CAMERA_INFO_TOPIC}" \
  target_frame:="${GRAVITY_ALIGNED_LOCAL_FRAME:?set GRAVITY_ALIGNED_LOCAL_FRAME}" \
  input_is_motion_compensated:=true \
  config_file:=/ros2_ws/install/share/ddlzd_ros/config/live_fusion.yaml \
  use_rviz:=true
```

Use `input_is_motion_compensated:=false` only when that input preserves the Ouster `original` `t` field and the required scan-duration TF is available. Use `true` only when the upstream component has already registered every point into `target_frame` at the message timestamp.

To run RViz on a separate workstation instead of inside the Jetson container, set `use_rviz:=false`, source a ROS 2 Humble environment on the workstation, use the same CycloneDDS domain/network configuration, and run:

```bash
rviz2 -d /path/to/live_landing_zones.rviz \
  -f "${GRAVITY_ALIGNED_LOCAL_FRAME:?set GRAVITY_ALIGNED_LOCAL_FRAME}"
```

In RViz, `Live registered point cloud` displays the rolling cloud actually evaluated by the detector. `Landing-zone categories` displays exact 2.5 m circles: green is `SAFEST`, orange is `SAFE`, red is `RISKY`, and gray is `UNKNOWN`. Both topics update on every completed live detection snapshot.

Useful verification commands are:

```bash
ros2 topic hz /ouster/points
ros2 topic echo /landing_zone/live_landing_zone/zones --once
ros2 topic hz /landing_zone/live_landing_zone/local_cloud
ros2 topic hz /landing_zone/live_landing_zone/markers
ros2 lifecycle get /landing_zone/live_landing_zone
ros2 topic echo /diagnostics --once
```

## Outputs

- `/landing_zone/live_landing_zone/zones`: `ddlzd_msgs/LandingZoneArray` with every metric and rejection reason.
- `/landing_zone/live_landing_zone/markers`: green, orange, red, or gray fixed-radius circles and stable IDs.
- `/landing_zone/live_landing_zone/debug_image`: RGB semantic points and projected candidate outlines.
- `/landing_zone/live_landing_zone/local_cloud`: the registered rolling cloud used by the detector.
- `/diagnostics`: freshness, fusion validity, cloud size, candidate count, and processing latency.

Marker colors are green `SAFEST`, orange `SAFE`, red `RISKY`, and gray `UNKNOWN`. Markers have finite lifetimes and the node publishes `DELETEALL` on deactivation.

## Calibration protocol

Calibration is a measured input, not a tunable visual alignment:

1. Calibrate camera intrinsics over the operational focus and resolution; publish the resulting rectified image and `CameraInfo`.
2. Estimate LiDAR-to-gimbal-base and camera-to-gimbal-camera rigid transforms using a joint target visible in both modalities. Report translation and rotation residuals.
3. Publish Gremsy encoder attitude as timestamped TF using the gimbal's documented rotation convention. Do not derive gimbal attitude from image appearance.
4. Estimate camera, LiDAR, vehicle-state, and gimbal clock offsets from a dynamic sequence. Correct offsets at the source or in the timestamping bridge.
5. Validate projection with held-out targets across gimbal angles and distances. Report pixel reprojection error and its metric footprint error.

The TF buffer must contain transforms over the complete Ouster frame interval. A transform only at receipt time is insufficient for an airborne scan.

## Experimental validation

Record raw topics before changing thresholds. Split recordings by site and flight, not by individual frame, to avoid spatial and temporal leakage.

At minimum, report:

- Candidate detection precision/recall against independently annotated 2.5 m footprints.
- False-safe rate for `SAFEST` and `SAFE`; this is the primary safety error.
- Category confusion matrix including `UNKNOWN`.
- Calibration reprojection error and timing-offset sensitivity.
- Performance as a function of altitude, range, incidence angle, surface type, illumination, gimbal angle, vehicle speed, and point density.
- Temporal stability, time-to-detection, stale-output rate, and end-to-end latency distributions.
- Ablations for LiDAR-only, RGB-only scoring terms, no deskew, and no temporal filtering.

Threshold selection and final evaluation must use different sites/flights. Confidence intervals should be computed by flight or site-level bootstrap rather than treating correlated frames as independent samples.

## Known scope

The RGB model is an explainable, zero-training feature model carried over from the repository's current offline implementation. It is not a learned land-cover classifier. It should be compared with a trained semantic model under the same held-out protocol before selecting the production research configuration. Regardless of the RGB model, LiDAR geometry remains the hard rejection authority.
