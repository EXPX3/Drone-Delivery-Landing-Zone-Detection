# Synchronized landing-zone bag replay viewer

This is a separate implementation from the static risk-ranked HTML and the MP4
projection script. It leaves both working versions unchanged and creates:

`pcds/region_growing_landing_zones_live_bag_replay.html`

The local replay server reads the ROS 2 bag with its original timestamps and updates:

- all safest, safe, risky, and top-30 landing-zone footprints;
- the UAV's accumulated 3D trajectory and current position;
- the camera frustum and its image-boundary footprint on the point-cloud ground plane;
- the current camera image, including the same in-frame LZ projection;
- a dynamic `Frustum-visible LZ highlight` layer computed independently in 3D;
- playback time, frame number, and visible-solution count.

For every frame, the server performs two independent classifications. The image set
uses distortion-aware OpenCV pixel projection. The map set transforms each 3D zone
center into camera coordinates and applies the Brown-Conrady lens model independently
to test the exact distorted image boundary. Both calculations use the same declared
partial-footprint policy: the center must be in-frame and at least 80 percent of the
metric landing-zone circle must remain in front of the camera. Neither result is
changed to agree with the other. The sidebar reports both ID sets,
`Independent set match: YES/NO`, and any
image-only or frustum-only candidates. It also accumulates checked-frame and
mismatch-frame counts over the complete replay and retains the first ten mismatch
examples in the state API.

The independently selected 3D landing-zone footprints are redrawn as thick light
pink (`#FFB6C1`) circles. The landing zones themselves do not move; the pink subset
changes as the camera frustum moves across the static point cloud. Image overlays
retain their original risk colors so the two calculations remain visually distinct.
Pink circles remain in the point cloud's orthometric elevation datum; the configured
geoid-to-GNSS altitude correction is applied only during camera-frame visibility
calculation and is not added to their displayed elevation.

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
