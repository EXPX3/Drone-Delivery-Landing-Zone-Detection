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

## Odometry-seeded scan optimization

LIO-SAM is not used for this bag because its upstream documentation recommends at
least 200 Hz IMU, while the recorded IMU rate is 35 Hz. The implemented next stage
uses KISS-ICP 1.2.3 for point-timestamp deskew and scan-to-local-map registration,
with recorded onboard odometry as a motion prior. A translation/rotation innovation
gate prevents weakly observable flat-ground ICP from corrupting the map, and accepted
map-to-odometry corrections are temporally filtered.

Create the pinned environment:

```bash
/usr/bin/python3 -m venv /home/gvb/Documents/robotspace/venvs/ddlzd_mapping
/home/gvb/Documents/robotspace/venvs/ddlzd_mapping/bin/pip install \
  -r scripts/bag_lio_mapping/requirements.txt
```

Reconstruct the complete map from optimized, deskewed scans:

```bash
/home/gvb/Documents/robotspace/venvs/ddlzd_mapping/bin/python \
  scripts/bag_lio_mapping/run_kiss_icp_mapping.py \
  --bag /media/gvb/ssd24ubuntu2/bags/Bagfilles_Feb13_2026/recovered_bag
```

The output is `pcds/bag_only_kiss_icp_reconstructed.pcd`; optimized poses and full
diagnostics are saved beside it. Bayern data is never read by this command.

Fuse repeated observations into 10 cm probabilistic surfels:

```bash
/home/gvb/Documents/robotspace/venvs/ddlzd_mapping/bin/python \
  scripts/bag_lio_mapping/fuse_surfel_map.py
```

Each surfel stores its centroid, observation count, full symmetric covariance,
surface normal, and normal-direction standard deviation. Cells with fewer than five
observations remain in the PCD to preserve sparse small objects, but receive no
fabricated normal. The direct reconstructed PCD remains the master artifact.

Generate a smoother inspection product that merges nearby layers into 20 cm cells
and keeps only cells observed at least three times:

```bash
/home/gvb/Documents/robotspace/venvs/ddlzd_mapping/bin/python \
  scripts/bag_lio_mapping/fuse_surfel_map.py \
  --voxel-size 0.20 --minimum-output-observations 3 \
  --output pcds/bag_only_kiss_icp_supported_surfels_20cm.pcd \
  --diagnostics pcds/bag_only_kiss_icp_supported_surfels_20cm.json
```

This is a statistically supported visualization/map product. It suppresses isolated
noise but can also remove small or transient objects that were observed fewer than
three times; it does not repair trajectory misregistration.

Visualize either output:

```bash
scripts/lz_rviz_viewer/run_mapping_audit_viewer.sh \
  pcds/bag_only_kiss_icp_reconstructed.pcd

scripts/lz_rviz_viewer/run_mapping_audit_viewer.sh \
  pcds/bag_only_kiss_icp_surfels_10cm.pcd
```

### Measured result

The complete run processed all 3,436 scans and reconstructed 7,983,601 points at
22.3 scans/s. The strict 0.20 m innovation gate rejected 2,157 scan corrections;
therefore this is a conservative odometry-anchored optimized map, not autonomous
SLAM. Fusion produced 3,135,787 surfels. Of these, 358,139 have at least five
observations and valid covariance-derived normals; their median normal-direction
spread is 0.0127 m and P95 is 0.0220 m.

The same internal 0.25 m XY-column diagnostic gives 0.829 m median thickness for
this reconstruction, versus 0.796 m for the original single-pose control. Because
columns also include real trees, people, boxes, cars and facade structure, this is
not an absolute-error metric; nevertheless, it does **not** demonstrate improved map
sharpness. The surfel's 0.0127 m within-cell statistic must not be confused with
whole-map registration accuracy.

The next stage is global pose-graph optimization with loop closure and GNSS factors,
followed by another reconstruction from the globally optimized poses. Bayern remains
held out until that map is frozen and evaluated.
