# Synchronized landing-zone bag replay viewer

This is a separate implementation from the static risk-ranked HTML and the MP4
projection script. It leaves both working versions unchanged and creates:

`pcds/region_growing_landing_zones_live_bag_replay.html`

The local replay server reads the ROS 2 bag with its original timestamps and updates:

- all safest, safe, risky, and top-30 landing-zone footprints;
- the UAV's accumulated 3D trajectory and current position;
- the camera frustum and its image-boundary footprint on the point-cloud ground plane;
- the current camera image, including the same in-frame LZ projection;
- playback time, frame number, and visible-solution count.

The exact original viewer colors are preserved: `green` (`#008000`) for safest,
`orange` (`#FFA500`) for safe, `red` (`#FF0000`) for risky, and `lightgreen`
(`#90EE90`) for ranks 1-30.

## Run

From the repository root:

```bash
scripts/lz_bag_replay_viewer/run_replay.sh
```

The first pass through the 31 GB bag builds the synchronized GPS/TF registration.
While that runs, the browser reports `Preparing bag`. Open:

```bash
google-chrome http://127.0.0.1:8765
```

The page has pause, restart, and `0.5x` to `4x` rate controls. A different bag and
port can be supplied as positional arguments:

```bash
scripts/lz_bag_replay_viewer/run_replay.sh /path/to/ros2_bag 8877
```

## Pose model

The camera is modeled as rigidly attached to the UAV. Position, altitude, roll,
pitch, and heading come from the timestamp-interpolated `map -> base_link` TF.
The recorded static `base_link -> altax_center_flu -> camera_body_flu` lever arm is
composed with the fixed nadir optical rotation from
`scripts/config/lz_camera_projection.yaml`. The dynamic gimbal TF is intentionally
not used.

The image footprint is obtained by undistorting all four image corners, rotating
their camera rays into EPSG:25832 coordinates, and intersecting them with the median
landing-zone elevation. This is a display footprint, not an occlusion model.

The calibration remains marked provisional until the exact 1280x720 intrinsic
calibration and surveyed camera mounting transform are available.
