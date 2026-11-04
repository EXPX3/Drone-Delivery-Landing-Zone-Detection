# Portable Replay Setup

This guide reproduces the recorded-bag RViz landing-zone demonstration on another
Ubuntu computer. It covers replay and analysis only; `replay_node.py` is not an
onboard live-flight node and must not command a UAV.

## 1. Host requirements

- Ubuntu 22.04 with an X11 session, or XWayland enabled;
- Docker Engine with permission to run `docker` as the current user;
- at least 12 GB free disk space in addition to the ROS bag;
- optional NVIDIA driver and NVIDIA Container Toolkit for GPU acceleration.

Confirm the host first:

```bash
docker version
echo "$DISPLAY"
docker info --format '{{json .Runtimes}}'
```

The launcher automatically uses NVIDIA only when both `nvidia-smi` and Docker's
`nvidia` runtime are available. Override this with `DDLZD_GPU=on` or
`DDLZD_GPU=off`.

## 2. Clone the exact implementation

```bash
git clone https://github.com/EXPX3/Drone-Delivery-Landing-Zone-Detection.git
cd Drone-Delivery-Landing-Zone-Detection
git switch sematic_segment_obstacles_dynamic_lz
git pull --ff-only
```

The `os1` tag points to the reviewed OS1-compatible checkpoint. The branch may
contain later fixes; use the branch for normal testing.

## 3. Install the runtime image

The launcher defaults to the project runtime image `cargopack:2026`. That image is
not published by this repository. Transfer it from the validated development
computer:

```bash
# On the source computer
docker save cargopack:2026 | gzip > cargopack-2026.tar.gz

# On the destination computer
gunzip -c cargopack-2026.tar.gz | docker load
docker image inspect cargopack:2026 >/dev/null
```

Alternatively, set `DDLZD_RVIZ_IMAGE` to a compatible ROS 2 Humble image containing
RViz2, `image_view`, `rosbag2_py`, OpenCV, NumPy, PyYAML, PCL support, and the
semantic model dependencies. `docker/Dockerfile.live_demo_cargopack` targets an
ARM64 live-UAV environment and is not a replacement for the validated x86 replay
image.

## 4. Copy untracked experiment artifacts

The `pcds/` directory is intentionally Git-ignored because it contains large and
site-specific data. Copy these paths while preserving their names:

```text
pcds/696_5328_cropped_by_bayernatlas_20260812084517.pcd
pcds/696_5328_cropped_ground_only.pcd
pcds/region_growing_landing_zones_max10000_radius2_5_to3_risk_ranked.csv
pcds/696_5328_cropped_by_bayernatlas_20260812084517_satellite_overlay_dop20.png
pcds/semantic_experiment/models/yoloe-11s-seg.pt
```

The orthophoto is required only when orthophoto mode is `on`. The YOLOE weights are
required only when camera fusion is enabled. Recorded-Velodyne replay additionally
requires:

```text
pcds/bag_only_strip_optimized_poses.npz
```

OS1 replay deliberately does not use that Velodyne pose archive. Transfer data with
`rsync -a` or another integrity-preserving method, then compare checksums between
computers:

```bash
sha256sum pcds/696_5328_cropped_by_bayernatlas_20260812084517.pcd \
  pcds/696_5328_cropped_ground_only.pcd \
  pcds/region_growing_landing_zones_max10000_radius2_5_to3_risk_ranked.csv
```

The ROS 2 bag must be a directory containing `metadata.yaml` and at least one
`*.db3` file.

## 5. Run LiDAR-only or fused replay

Set paths once per shell instead of relying on machine-specific mount names:

```bash
export DDLZD_BAG=/absolute/path/to/ros2_bag
export DDLZD_RVIZ_IMAGE=cargopack:2026
```

For CPU-only LiDAR replay, set `DDLZD_GPU=off` and use camera mode `off`. The
checked-in semantic configuration selects CUDA device `0`; fused camera operation
therefore requires a CUDA-capable runtime unless a copied replay YAML explicitly
changes `semantic_segmentation.device` to a supported CPU setting.

For different artifact locations or a different mapped site, set these variables;
the launcher mounts each host path into the container:

```bash
export DDLZD_STATIC_PCD=/data/site/static_map.pcd
export DDLZD_GROUND_PCD=/data/site/ground_map.pcd
export DDLZD_ZONES_CSV=/data/site/landing_zones.csv
export DDLZD_ORTHOPHOTO_FILE=/data/site/orthophoto.png
export DDLZD_SEMANTIC_WEIGHTS=/data/models/yoloe-11s-seg.pt
```

OS1, LiDAR-only:

```bash
scripts/lz_rviz_viewer/run_rviz_replay.sh \
  "$DDLZD_BAG" 1.0 12 camera_lidar_correlated_only on os1 off
```

OS1 with automatic camera fusion when the configured image topic exists:

```bash
scripts/lz_rviz_viewer/run_rviz_replay.sh \
  "$DDLZD_BAG" 1.0 12 camera_lidar_correlated_only on os1 auto
```

Use camera mode `required` during a fused-data validation run; it fails clearly if
the configured image topic is absent. In `auto` or `off`, camera availability never
gates LiDAR mapping, occupancy, or landing-zone updates.

## 6. Adapt topics and calibration

Do not edit code for a new UAV. Copy the OS1 profile and main YAML, then change:

- LiDAR topic and `PointCloud2.header.frame_id`/TF connectivity;
- GNSS, TF, and optional image topics;
- OS1-to-body and camera-to-body extrinsics;
- camera matrix, distortion, and calibration resolution;
- static-map registration for the deployment site.

Select those copies explicitly:

```bash
export DDLZD_REPLAY_CONFIG=/absolute/path/to/site_replay.yaml
export DDLZD_SENSOR_PROFILE=/absolute/path/to/os1_profile.yaml
```

The checked-in Bayern map, orthophoto, landing zones, and registration are valid only
for the original test site. A new site requires its own georeferenced point cloud,
ground cloud, orthophoto, candidate CSV, and registration calibration.

Run preflight while sensors are publishing:

```bash
source /opt/ros/humble/setup.bash
python3 scripts/semantic_lz/os1_preflight.py \
  --points-topic /ouster/points \
  --gps-topic /mavros/global_position/global \
  --base-frame base_link \
  --camera-mode off \
  --duration 15
```

Use `--camera-mode required --image-topic /your/image/topic` when validating fusion.
`auto` reports whether images were observed but does not fail a LiDAR-only system.

## 7. Important limitations

- The replay performs an offline bag scan for TF/GNSS registration before playback.
- The OS1 profile accepts Ouster per-point `t`, `time`, or `timestamp`, but accurate
  deskewing still requires calibrated timing, extrinsics, and a validated trajectory.
- The semantic layer assists class localization; geometric LiDAR occupancy remains
  authoritative for landing safety.
- A real-aircraft deployment requires a dedicated live ROS 2 node, bounded latency,
  watchdogs, stale-data handling, hardware-in-the-loop testing, and an independent
  flight-safety case.
