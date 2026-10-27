# Camera-LiDAR Localization Audit

## Scope

This audit records the observed final state at RViz frame `3006`
(`1770993689718362016 ns`). It compares persistent camera-only ground
projections with camera-correlated LiDAR tracks on the georeferenced Bayern
DOP20 orthophoto.

![Camera and LiDAR detection audit](../pcds/semantic_experiment/camera_lidar_final_state_audit.png)

The map uses local East/North metric coordinates, a 5 m grid, a 10 m scale bar,
and ROS axis colors: X/East is red and Y/North is green. Hollow symbols and thin
95% covariance contours are camera-only proposals. Filled squares and solid
contours are camera-correlated LiDAR tracks. White lines connect each camera
proposal to its nearest same-class LiDAR track.

## Results

| Class | Camera only | Correlated LiDAR | Excess camera tracks | Median nearest error | RMSE | Maximum |
|---|---:|---:|---:|---:|---:|---:|
| Cargo box | 9 | 3 | 6 | 4.80 m | 7.97 m | 18.66 m |
| Person | 1 | 1 | 0 | 0.75 m | 0.75 m | 0.75 m |
| Vehicle | 3 | 1 | 2 | 5.15 m | 7.08 m | 11.11 m |

The five best one-to-one, class-matched camera/LiDAR object pairs have a median
error of `0.72 m`, RMSE of `0.71 m`, and maximum error of `0.78 m`. The three
cargo core errors are `0.59 m`, `0.72 m`, and `0.78 m`.

## Finding

The camera projection has an accurate core near `0.7 m`, but also large,
viewpoint-dependent displaced modes. Its error is therefore correlated and
non-Gaussian; a single covariance ellipse does not describe the complete error
distribution. Camera semantics are useful for class assignment, while LiDAR
geometry should remain authoritative for obstacle location and landing-zone
safety.

The LiDAR tracks are an operational reference, not surveyed ground truth. The
person was absent from the final live marker frame, so its confirmed full-bag
camera-correlated LiDAR track is used and identified as such in the JSON data.

## Reproduction

Capture one marker frame and render the audit:

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /ddlzd/replay_markers --once > /tmp/ddlzd_final_markers.yaml

/home/gvb/Documents/robotspace/venvs/ddlzd_mapping/bin/python3 \
  scripts/semantic_lz/render_camera_lidar_audit.py \
  --markers /tmp/ddlzd_final_markers.yaml
```

Machine-readable results are stored in
`pcds/semantic_experiment/camera_lidar_final_state_audit.json`.
