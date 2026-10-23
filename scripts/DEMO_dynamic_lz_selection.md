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
translation:  [ 0.007155, -0.107244, -1.638995 ] m
roll/pitch/yaw: approximately [ -0.211, 1.475, 0.036 ] degrees
median residual: 0.2443 m
75th percentile residual: 0.3177 m
```

The correction is stored under `registration.live_lidar_to_static` in
`scripts/config/lz_camera_projection.yaml` and is applied around the static-cloud
origin after map-to-orthometric conversion.

## Configuration

Dynamic parameters are under `dynamic_occupancy` in
`scripts/config/lz_camera_projection.yaml`:

```yaml
resolution_m: 0.75
confirmation_hits: 3
maximum_lidar_range_m: 80.0
point_stride: 2
publish_hz: 2.0
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

Expected result: 14 tests pass.

## Stop

```bash
docker stop ddlzd-lz-rviz
```
