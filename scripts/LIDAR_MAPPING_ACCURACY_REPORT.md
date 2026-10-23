# LiDAR Mapping Accuracy and Layering Audit

## Data and method

The audit processed all 3,436 `/velodyne_points` scans from the recovered bag.
Every fourth return was transformed through the recorded TF chain and compared with
the Bayern classified-ground cloud. The output map uses 0.20 m XY cells and stores
one final point per changed cell. Repeated observations are retained as statistics,
not duplicated as visible map layers.

The bag starts and ends with the UAV grounded. A robust per-scan vertical correction
was estimated only from unchanged terrain. Returns 0.30-5.0 m above that corrected
terrain, at least 0.30 m from the original full Bayern cloud, and observed in at
least three scans were exported separately as `change_only_obstacles.pcd`. This
separates the intentionally placed boxes, person, car, and vegetation changes from
the ground-accuracy calculation.

## Results

| Metric | Original transformed points | Ground-anchored map |
|---|---:|---:|
| Ground RMSE | 0.883 m | 0.127 m |
| Median vertical bias | 0.821 m | 0.004 m |
| Robust scatter (NMAD) | 0.516 m | 0.118 m |
| 95th percentile absolute residual | 1.410 m | 0.256 m |
| Median repeated-cell standard deviation | 0.299 m | 0.103 m |
| Median repeated-cell peak-to-peak layer | 0.982 m | 0.400 m |
| 95th percentile peak-to-peak layer | 1.541 m | 0.568 m |

The update-only ground map contains 170,600 cells. The change-only map contains
66,448 confirmed 0.20 m voxels. The horizontal GNSS/TF registration remains strong:
0.077 m RMSE and 0.030 m median error.

The first stationary 20 scans are substantially cleaner: 0.121 m uncorrected ground
RMSE, 0.017 m corrected RMSE, and 0.003 m median repeated-cell peak-to-peak spread.
The large layer develops during flight and disappears when the UAV returns.

## Mount and vibration assessment

The evidence does not support hard-mount vibration as the dominant cause:

- The stationary hard-mounted sensor produces centimetre-level repeatability.
- The flight error is a smooth vertical bias, reaching about 1.2 m and returning
  near zero at the starting location. Random mechanical vibration would primarily
  increase high-frequency spread rather than produce this trajectory-dependent bias.
- Vertical GPS and TF positions agree to millimetres during flight, so the mismatch
  occurs after the vehicle state estimate, in LiDAR timing/extrinsic/map projection.
- Median bias has only weak correlation with angular speed (`r = 0.25`). Apparent
  scan tilt has weak correlation with angular speed (`r = 0.18`) and acceleration
  magnitude deviation (`r = 0.02`).

The strongest likely contributors are use of one timestamp for an entire rotating
scan (no point-level deskew), residual LiDAR-to-body attitude calibration, and a
small-patch ICP correction originally fitted from only the first 240 mostly
stationary scans. Disabling that ICP does not remove layering: median repeated-cell
peak-to-peak spread remains approximately 1.06 m.

The recorded raw IMU rate is 35.71 Hz, giving a 17.85 Hz Nyquist limit. Its dominant
recorded energy is below 1 Hz and represents vehicle motion. Typical motor/propeller
vibration cannot be established or excluded from this bag because the recorded IMU
cannot resolve those frequencies.

## Changed objects

The fine change map retains object returns hidden by the combined 0.75 m occupancy
display. A car-scale connected component spans approximately 3.8 x 5.8 x 1.85 m.
Narrow components approximately 0.15-0.38 m wide and 0.95-0.99 m tall are compatible
with sparse person or box surfaces. Changed trees and residual low terrain voxels
form a much larger connected component, so synchronized camera detections should be
used for semantic labels rather than assigning labels from geometry alone.

## Reproduction

```bash
cd /home/gvb/Documents/robotspace/DDLZD_ws/Drone-Delivery-Landing-Zone-Detection
docker run --rm --name ddlzd-mapping-audit \
  -v "$PWD:/workspace" \
  -v /media/gvb/ssd24ubuntu1/bags/Bagfilles_Feb13_2026/recovered_bag:/bag:ro \
  -w /workspace --entrypoint bash cargopack:2026 -lc \
  'source /opt/ros/humble/setup.bash && \
   python3 scripts/analyze_lidar_mapping_accuracy.py --bag /bag'
```

Outputs are generated in `pcds/lidar_mapping_accuracy/`:

- `mapping_accuracy_report.json`: aggregate accuracy and IMU statistics;
- `per_scan_metrics.csv`: time-resolved bias, RMSE, tilt, and motion values;
- `direct_corrected_pointcloud.pcd`: individual corrected LiDAR returns without
  voxelization or per-cell averaging;
- `update_only_ground_map.pcd`: ground-anchored update-only map;
- `change_only_obstacles.pcd`: confirmed scene changes without baseline ground.

Inspect all map layers interactively in RViz:

```bash
scripts/lz_rviz_viewer/run_mapping_audit_viewer.sh
```

Use the RViz Displays panel to toggle the Bayern reference, corrected update-only
ground, and confirmed scene-change layers independently.
The direct-return layer uses a fixed local-height gradient from blue near the ground
through yellow to red at 3 m and above. Fixed bounds keep person, box, and car heights
visually distinct even when tall trees are present.

The viewer defaults to an exact, non-downsampled 100 m radius around the grounded bag
start (`696496.705, 5328128.777` UTM), retaining individual returns and nearby
person/box/car-height changes without voxelization.
Pass `--radius 0` to `mapping_audit_viewer.py` for the complete eight-million-point map.

For a definitive vibration test, record the LiDAR packet firing times and a rigidly
mounted IMU at several hundred hertz or above. Then deskew each return using the
continuous-time body pose and compare maps with and without deskew using this same
static-ground reference.
