# Bag-Only LiDAR Mapping

This directory contains the mapping path that uses only sensors recorded onboard the
UAV. Bayern point clouds are excluded from estimation and reserved for held-out
accuracy evaluation.

## Current stage

`build_deskewed_map.py` is the reproducible baseline before tightly coupled LIO. It:

1. reads the recorded `map -> base_link` trajectory and static LiDAR chain;
2. reads the organized VLP-16 cloud with `ring` and per-point `time` fields;
3. interpolates the body pose at every retained firing time;
4. transforms each return through the calibrated frame chain;
5. streams a binary PCD without voxel fusion or Bayern correction.

Run a short verification:

```bash
docker run --rm \
  -v "$PWD:/workspace" \
  -v /media/gvb/ssd24ubuntu2/bags/Bagfilles_Feb13_2026/recovered_bag:/bag:ro \
  -w /workspace --entrypoint bash cargopack:2026 -lc \
  'source /opt/ros/humble/setup.bash && \
   python3 scripts/bag_lio_mapping/build_deskewed_map.py --bag /bag --max-scans 100'
```

Remove `--max-scans 100` for the complete baseline map. Outputs default to:

- `pcds/bag_only_deskewed_map.pcd`
- `pcds/bag_only_deskewed_map_diagnostics.json`

This baseline is not called final SLAM: it still inherits the onboard trajectory and
does not jointly optimize IMU, LiDAR residuals, GNSS, or loop closures.

Generate the matched single-pose control with the same points and filtering:

```bash
python3 scripts/bag_lio_mapping/build_deskewed_map.py --bag /bag \
  --disable-deskew \
  --output pcds/bag_only_single_pose_control.pcd \
  --diagnostics pcds/bag_only_single_pose_control_diagnostics.json
```

Compare the two matched maps without consulting Bayern data:

```bash
python3 scripts/bag_lio_mapping/compare_maps.py
```

This writes `pcds/bag_only_deskew_comparison.json`. The correction-distance metric
quantifies how much per-point deskew moved each return. The repeated-column metric is
only an internal thickness proxy: trees, boxes, the person, the car and other genuine
vertical structure also widen an XY column. It is not an absolute-accuracy result.

## Result from this bag

Both complete maps contain 8,219,083 points from all 3,436 scans. Deskew moved a
matched point by 0.052 m median and 0.597 m at P95. It did not improve the current
all-structure column-thickness proxy (0.802 m deskewed versus 0.796 m control at the
median), so this baseline is evidence for the next calibration/LIO stage, not a
validated corrected map. See `scripts/BAG_ONLY_LIDAR_MAPPING_REPORT.md`.
