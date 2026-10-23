# Dynamic Landing-Zone Selection Demonstration

## Purpose

Branch `dynamic_lz_selection` extends the synchronized RViz replay with a sparse,
live occupancy map built from `/velodyne_points`. The Bayern point cloud initializes
the map, and registered LiDAR returns add confirmed occupancy while the bag plays.

The initial 1,356 risk-ranked candidates remain the terrain-geometry source of truth.
Live occupancy conservatively updates them:

- A newly occupied voxel inside a landing radius blocks that candidate.
- A newly occupied voxel inside the outer 2 m GNSS/wind margin keeps the candidate
  active but downgrades it to `risky`.
- Baseline voxels cannot invalidate baseline candidates.
- A live voxel requires three observations before affecting a candidate.
- Only obstacles 0.35-20 m above the landing surface affect clearance.
- Blocked candidates are removed from camera projection and shown as gray crosses.

The live sensor does not create new flat candidates from obstacle endpoints. Adding
new candidates requires rerunning region-growing surface analysis on an accumulated
registered surface cloud; treating endpoint occupancy alone as a landing surface
would not be geometrically valid.

## Start

```bash
findmnt -S /dev/nvme1n1p1 >/dev/null || udisksctl mount -b /dev/nvme1n1p1

cd /home/gvb/Documents/robotspace/DDLZD_ws/Drone-Delivery-Landing-Zone-Detection
git switch dynamic_lz_selection
docker rm -f ddlzd-lz-rviz 2>/dev/null || true
scripts/lz_rviz_viewer/run_rviz_replay.sh
```

Allow about 60-90 seconds for the one-time TF/GNSS registration scan. Follow status
from another terminal:

```bash
docker logs -f ddlzd-lz-rviz
```

## RViz Layers

- `Point Cloud`: source PCD rendered lightly for geometric context.
- `Dynamic Octomap / baseline_occupancy`: initial voxel occupancy.
- `Dynamic Octomap / live_occupancy`: confirmed LiDAR additions.
- `Landing Zones`: current active categories and blocked-zone crosses.
- `Live Camera Geometry`: trajectory, UAV, frustum, footprint, and camera-visible LZs.

The UAV sphere and trajectory use GNSS interpolated at the exact camera-image
timestamp. The camera/frustum use the recorded TF chain. A magenta residual line
connects the GNSS UAV position to the TF-derived base position, and the image status
reports the current horizontal `GNSS/TF` residual. The frame number and ROS timestamp
shown above the UAV are identical to those burned into the camera image.

Frustum corner rays are intersected iteratively with the classified Bayern ground
cloud, not a single flat elevation. Each resulting 3D corner is verified to reproject
to the corresponding camera-image corner under the same intrinsic/distortion model.
If identifiable image features still disagree with the footprint, the remaining
source is the provisional camera calibration rather than timestamp or ray geometry.

Voxel color follows altitude: blue at low elevations through cyan and yellow to red
at high elevations. Landing-zone rings have a 0.65 m RViz-only visual lift so voxel
faces do not hide them. Detection and camera projection retain the actual surface Z.

The image viewer reports active/blocked candidate counts and processed LiDAR frames.
Its projected circles use the same current candidate collection as RViz.

## Registration

The horizontal UTM-to-bag-map fit uses 2,022 synchronized GNSS/TF pairs and has
0.077 m RMSE. A robust point-to-point ICP fit of 240 Velodyne frames to the static
cloud supplies the remaining live-LiDAR correction:

```text
translation:  [ 0.107891, -0.092773, 2.272681 ] m
roll/pitch/yaw: approximately [ -0.211, 1.475, 0.036 ] degrees
median residual: 0.2443 m
75th percentile residual: 0.3177 m
```

The correction is stored under `registration.live_lidar_to_static` in
`scripts/config/lz_camera_projection.yaml` and is applied around the static-cloud
3D median after map-to-orthometric conversion. The RViz display origin separately
keeps Z at zero so native elevations remain visible; it must not be used as the ICP
rotation pivot.

The bag starts and ends with the UAV grounded. Comparing all 12,132 GNSS samples
with the Bayern ground cloud showed that the former 45.0 m ellipsoid-to-DHHN
estimate left the minimum trajectory point 3.913 m above terrain. The configured
vertical offset is therefore 48.913 m. At the first camera frame, the GPS trajectory
is 0.411 m above terrain and the TF `base_link` is 0.258 m above terrain, consistent
with the body/GNSS reference point of a grounded UAV. The stored ICP translation was
re-expressed for this datum change, so static/live point-cloud alignment is unchanged.

## Configuration

Dynamic parameters are under `dynamic_occupancy` in
`scripts/config/lz_camera_projection.yaml`:

```yaml
resolution_m: 0.75
confirmation_hits: 3
maximum_lidar_range_m: 80.0
point_stride: 2
publish_hz: 2.0
zone_publish_hz: 1.0
gps_wind_margin_m: 2.0
obstacle_min_height_m: 0.35
vertical_clearance_m: 20.0
zone_visual_lift_m: 0.65
```

## Tests

```bash
/home/gvb/Documents/robotspace/venvs/praktikum_flightplanning_venv/bin/python \
  -m unittest discover -s scripts/tests -v
```

Expected result: 17 tests pass.

## Synchronization Evidence

Both `/ddlzd/camera_overlay` and every marker in `/ddlzd/replay_markers` carry the
same source image timestamp. A 30-pair runtime check found no unmatched samples. With
the image published before its marker bundle, image-minus-marker arrival skew was
`-2.8` to `+10.4 ms` with a `-1.6 ms` median. Rendering in RViz and `image_view`
remains asynchronous, but neither stream represents a different bag frame.

In the same runtime check, independently interpolated GNSS and TF-derived UAV
horizontal positions differed by approximately 0.0051 m. A larger apparent offset
between image content and mapped objects therefore belongs to camera-to-ground
calibration, not UAV georegistration. The camera calibration remains provisional.

## Stop

```bash
docker stop ddlzd-lz-rviz
```

## LiDAR Mapping Accuracy Audit

The offline audit builds a 0.20 m update-only ground map and measures direct-point
residuals, repeated-cell layering, apparent scan tilt, IMU correlation, and the
recorded IMU spectrum:

```bash
docker run --rm --name ddlzd-mapping-audit \
  -v "$PWD:/workspace" \
  -v /media/gvb/ssd24ubuntu1/bags/Bagfilles_Feb13_2026/recovered_bag:/bag:ro \
  -w /workspace --entrypoint bash cargopack:2026 -lc \
  'source /opt/ros/humble/setup.bash && \
   python3 scripts/analyze_lidar_mapping_accuracy.py --bag /bag'
```

Generated results are under `pcds/lidar_mapping_accuracy/`. The PCD contains only
one final point per changed XY cell; repeated observations are summarized in the
JSON and per-scan CSV instead of creating visible layers.

Measured results and the mount-vibration assessment are documented in
`scripts/LIDAR_MAPPING_ACCURACY_REPORT.md`. The pipeline also writes
`change_only_obstacles.pcd`, which excludes original Bayern geometry and keeps
confirmed above-ground changes in a separate 0.20 m map.
