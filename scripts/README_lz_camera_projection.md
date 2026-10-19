# Landing-zone camera projection

`project_lz_into_bag_camera.py` projects georeferenced landing-zone circles into
the recorded Gremsy camera stream. It uses the complete recorded transform chain,
interpolates TF at each image timestamp, and robustly registers UTM coordinates to
the ROS `map` frame using synchronized GNSS and `map -> base_link` poses.

The existing offline point-cloud/satellite viewer remains documented in
`README_pointcloud_crop_viewer.md`. It is independent of this camera projection.

## Inputs and outputs

Default inputs:

- Camera: `/GremsyVIO/image/cpu` (`1280x720`, approximately 30 Hz)
- Position: `/mavros/global_position/global`
- Pose: `/tf` and `/tf_static`
- Zones: the risk-ranked CSV, filtered to the top 30 `safest` zones
- Coordinate system: EPSG:25832 horizontal coordinates

Outputs:

- An MP4 with translucent, light-green landing-zone footprints
- A JSON diagnostics file containing registration error, trajectory baseline,
  skipped frames, and mean visible-zone count

## Calibration gate

The supplied configuration is intentionally marked `provisional`. The bag has no
`CameraInfo` message and no recorded `gimbal_link_flu -> camera optical` transform.
The known 1920x1080 camera matrix is scaled to 1280x720, and the standard FLU to
optical rotation is used, but these two assumptions must be checked against the
camera pipeline. The configured 45 m vertical datum correction is also an initial
value for converting Bayern orthometric LiDAR heights to GNSS altitude.

For a metrically defensible result, measure and update these fields in
`scripts/config/lz_camera_projection.yaml`:

- Camera matrix and distortion at the recorded resolution and zoom
- `gimbal_to_optical.translation_m` and `quaternion_xyzw`
- `lz_to_gnss_vertical_offset_m`
- Set `calibration.status: measured` only after validation

Provisional runs require an explicit flag and are watermarked in every frame.

## Run on the recorded bag

The existing extraction image contains ROS 2, OpenCV, NumPy, and the required
message definitions. From the repository root:

```bash
docker run --rm \
  --user "$(id -u):$(id -g)" \
  -e ROS_LOG_DIR=/tmp/ros_logs \
  -v "$PWD:/workspace" \
  -v /media/gvb/ssd24ubuntu/bags/Bagfilles_Feb13_2026/recovered_bag:/bag:ro \
  -w /workspace \
  ws_extract_images_or_vid_frombag:latest \
  bash -lc 'source /opt/ros/humble/setup.bash && \
    python3 scripts/project_lz_into_bag_camera.py \
      --bag /bag \
      --landing-zones pcds/region_growing_landing_zones_max10000_radius2_5_to3_risk_ranked.csv \
      --output pcds/lz_camera_projection.mp4 \
      --allow-provisional-calibration'
```

First run a short calibration sample before processing the full 346-second bag:

```bash
docker run --rm \
  --user "$(id -u):$(id -g)" \
  -e ROS_LOG_DIR=/tmp/ros_logs \
  -v "$PWD:/workspace" \
  -v /media/gvb/ssd24ubuntu/bags/Bagfilles_Feb13_2026/recovered_bag:/bag:ro \
  -w /workspace \
  ws_extract_images_or_vid_frombag:latest \
  bash -lc 'source /opt/ros/humble/setup.bash && \
    python3 scripts/project_lz_into_bag_camera.py \
      --bag /bag --start-s 120 --duration-s 10 --max-frames 60 \
      --output pcds/lz_camera_projection_calibration.mp4 \
      --allow-provisional-calibration'
```

Open the result:

```bash
xdg-open pcds/lz_camera_projection_calibration.mp4
```

Use `--categories safest,safe,risky` to render all risk classes, or change
`--max-zones` to control the candidate count. Green, orange, and red correspond to
`safest`, `safe`, and `risky`.

## Verification

The pure geometry tests run in the existing project environment:

```bash
/home/gvb/Documents/robotspace/venvs/praktikum_flightplanning_venv/bin/python \
  -m unittest discover -s scripts/tests -v
```

Do not use the projection for flight decisions until the provisional calibration
watermark is gone and the overlay has been validated against surveyed image points.
