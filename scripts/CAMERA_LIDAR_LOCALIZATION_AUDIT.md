# Camera-LiDAR Localization Audit

## Scope

This audit records the observed state at RViz frame `3006`
(`1770993689718362016 ns`). It compares camera-only ground projections and
camera-correlated LiDAR tracks independently against the provided survey
ground truth on the georeferenced Bayern DOP20 orthophoto.

![Camera and LiDAR ground-truth comparison](../pcds/semantic_experiment/camera_lidar_ground_truth_comparison.png)

Cargo naming matches `confirmed_objects_ground_truth_axes_v3.png`: Cargo 1 is
the middle box, Cargo 2 is the far-right box, and Cargo 3 is the left box.
Camera proposal modes are assigned to the nearest same-class survey center.
The map uses the survey X/Y axes in metres, ROS axis colors, north arrow and a
5 m scale bar. Hollow circles and dashed contours show camera-only estimates;
filled squares and dark solid contours show camera-correlated LiDAR estimates;
cyan crosses show survey centers.

## Results

| Object | Camera modes / observations | Camera center error | Camera mode RMSE | Camera maximum | Camera+LiDAR GT error | LiDAR P95 |
|---|---:|---:|---:|---:|---:|---:|
| Cargo 1, middle | 2 / 225 | 0.92 m | 1.11 m | 4.90 m | 0.12 m | 0.44 m |
| Cargo 2, far right | 3 / 213 | 0.77 m | 2.89 m | 18.74 m | 0.38 m | 0.43 m |
| Cargo 3, left | 4 / 186 | 1.11 m | 2.24 m | 11.03 m | 0.25 m | 0.32 m |
| Person 1 | 1 / 32 | 0.54 m | 0.54 m | 0.54 m | 0.20 m | 0.22 m |

`Camera center error` is the observation-count-weighted camera aggregate versus
ground truth. `Camera mode RMSE` preserves displaced proposal modes and weights
them by their observation counts. `Camera maximum` is the most displaced mode.
The person camera-correlated LiDAR estimate comes from its validated full-bag
track because the person is absent from the final live marker frame.

The vehicle is displayed for completeness, but no surveyed vehicle coordinate
was supplied. Its LiDAR track is therefore only an operational reference:
three camera modes have `4.25 m` RMSE and `11.11 m` maximum displacement from
that reference.

## Finding

The camera aggregate centers remain within `0.54–1.11 m` of survey truth because
dominant modes contain most observations. This hides severe minority modes:
Cargo 2 reaches `18.74 m` and Cargo 3 reaches `11.03 m`. The camera error is
therefore multimodal, correlated and non-Gaussian; one covariance around the
dominant center is insufficient. Camera-correlated LiDAR reduces cargo
ground-truth error to `0.12–0.38 m` and should remain authoritative for physical
location and landing-zone safety.

Nearest-ground-truth assignment is an evaluation procedure that uses supplied
truth and must not be used by a deployed system. It is appropriate here because
the historical camera proposal IDs cannot otherwise be associated reliably.

## Reproduction

Capture one marker frame and render the audit:

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /ddlzd/replay_markers --once > /tmp/ddlzd_final_markers.yaml

/home/gvb/Documents/robotspace/venvs/ddlzd_mapping/bin/python3 \
  scripts/semantic_lz/render_ground_truth_camera_lidar_comparison.py \
  --markers /tmp/ddlzd_final_markers.yaml
```

Machine-readable results are stored in
`pcds/semantic_experiment/camera_lidar_ground_truth_comparison.json`.
