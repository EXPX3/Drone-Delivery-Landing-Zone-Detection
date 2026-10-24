# Industry LiDAR Mapping Pipeline and Bag-Only Implementation

## Objective and separation of concerns

The production map must be generated solely from onboard LiDAR, IMU, UAV odometry,
GNSS, VIO, and TF data recorded in the ROS 2 bag. Bayern data is retained as held-out
ground truth: it may measure final error but must not influence deskewing, trajectory
estimation, scan registration, loop closure, or map fusion.

## Recorded evidence

The 346.7 s bag contains 3,436 organized Velodyne scans at 9.91 Hz. Each scan has 16
rings, approximately 29,184 points, and a float32 per-point time spanning about
`-99.5 ms` to `+1.3 ms` relative to its header. This is sufficient for point-level
deskewing. The static transform reports `altax_center_flu -> velodyne` translation
`[0.13, -0.24, -0.11] m` and a 90-degree Y rotation.

The onboard raw IMU topic is 35.0 Hz, local odometry is 21.0 Hz, GNSS is 35.0 Hz,
VIO is approximately 50 Hz, and raw MAVLink traffic is recorded. The PointCloud2
timing is suitable; the principal unresolved input limitation is whether raw MAVLink
contains a usable higher-rate IMU stream. The installed analysis image currently
lacks `mavros_msgs`, so that payload audit remains open.

## Standard production procedure

| Stage | Required work | Current status |
|---|---|---|
| Time synchronization | Establish clock domains and estimate LiDAR/IMU offset | Open |
| LiDAR intrinsics | Confirm VLP-16 model, channel calibration, RPM and range bias | Partially evidenced |
| Spatial calibration | Optimize 6-DoF LiDAR-to-IMU/body extrinsics | Open; recorded TF is only a prior |
| Point timing | Preserve ring and firing time for every return | Verified |
| Deskewing | Interpolate continuous pose at each point time | Baseline implemented |
| LIO front end | Joint LiDAR/IMU state estimation and scan-to-local-map matching | Next stage |
| Global graph | GNSS factors, loop closure and full trajectory optimization | Planned |
| Dynamic filtering | Exclude person, car, boxes and moving vegetation from registration | Planned |
| Map fusion | Fuse optimized scans as surfels/probabilistic points | Planned |
| Independent validation | Bayern point-to-plane error and surveyed checks | Existing evaluator, held out |

The ordering matters. Downsampling or large voxels may be used for compute and
display only after timing, calibration, deskewing and registration; they must not be
used to conceal layered scans.

## Implementation phases

### Phase 1: deterministic deskew baseline

Implemented under `scripts/bag_lio_mapping/`. It uses onboard TF poses and per-point
times, never accesses Bayern data, and writes a binary direct-point map plus
diagnostics. It establishes whether point-level motion compensation improves object
edges before changing the estimator.

### Phase 2: sensor contract and calibration

Decode raw MAVLink IMU messages if they provide a higher sampling rate; measure
timestamp latency and jitter; identify the exact Velodyne calibration; then estimate
temporal offset and 6-DoF LiDAR-to-IMU extrinsics with a continuous-time calibration
method. The 35 Hz ROS IMU cannot measure typical propeller vibration above 17.5 Hz.

### Phase 3: tightly coupled LIO

Integrate a proven engine such as FAST-LIO2 or LIO-SAM. Use deskewed raw points and
IMU propagation, local scan-to-map residuals, keyframes, and uncertainty-aware state
updates. Do not use unrestricted per-scan ICP against the Bayern cloud.

The recorded 35 Hz IMU does not meet LIO-SAM's upstream recommendation of at least
200 Hz. Consequently, the implemented intermediate stage uses KISS-ICP 1.2.3 with
correctly paired per-point timestamps and onboard odometry as a motion prior. A
0.20 m / 5 degree innovation gate and low-pass map-to-odometry correction prevent
flat-ground registration ambiguity from entering the reconstructed map. This is
scan-to-local-map optimization, not a claim of tightly coupled LIO.

### Phase 4: global consistency

Add GNSS and heading as weighted factors, detect the return to the starting area as a
loop closure, optimize the pose graph, and rebuild the point map from optimized poses.
GNSS altitude must not be treated as exact.

### Phase 5: dynamic filtering and map fusion

Reject transient objects from pose estimation using temporal occupancy and camera
evidence. Retain them in a separate change layer. Fuse static observations with
surfels or probabilistic statistics while preserving a full-resolution master map.

### Phase 6: held-out evaluation

Only after the bag-only map is frozen, register its global frame using onboard GNSS
and compare it with Bayern ground/full clouds. Report horizontal/vertical bias,
point-to-plane RMSE, repeated-surface thickness, normal consistency, edge sharpness,
loop-closure error, and error versus range, speed and angular rate.

## Claims and limitations

The current Bayern-assisted map removed the dominant per-scan vertical bias but did
not solve calibration: its corrected ground RMSE is 0.127 m and median repeated-cell
peak-to-peak thickness is 0.400 m. Horizontal duplication remains. The new bag-only
baseline must therefore be judged independently and is not presumed better until the
held-out evaluation is complete.

The bag is sufficient to begin an industry-style, self-contained mapping pipeline.
It is likely sufficient for a strong relative map because per-point LiDAR timing,
onboard motion, GNSS, VIO and a closing trajectory are present. It cannot yet support
a claim of survey-grade absolute accuracy or a conclusive high-frequency vibration
diagnosis without higher-rate IMU evidence and independent control measurements.

## Consolidated observations from the demonstration

- Large display voxels reduced the visible layering but did not remove it. Direct
  points exposed horizontally separated copies of surfaces and blurred object edges.
- The boxes and person are small relative to registration error and display voxels;
  the car remains more recognizable because it contributes a larger surface. These
  deliberately introduced changes must be excluded from static-map registration and
  retained in a separate change/occupancy layer.
- The prior Bayern-assisted vertical correction aligned ground height to 0.127 m
  RMSE, but horizontal duplication remained and repeated-cell peak-to-peak thickness
  was 0.400 m. This demonstrates vertical improvement, not complete calibration.
- The UAV was on the ground at bag start. A start pose above the Bayern ground is
  therefore a datum/extrinsic/timing error, not real hover height. GNSS ellipsoidal or
  MSL height, local-odometry origin and Bayern elevation datum must be reconciled only
  for global evaluation.
- A hard-mounted LiDAR can transmit vibration, but the 35 Hz recorded IMU has a
  17.5 Hz Nyquist limit and cannot establish rotor-frequency vibration. Layering can
  also arise from time offset, inaccurate extrinsics, rolling scan motion, odometry
  drift and weak registration.
- Camera image, frustum, UAV path and LiDAR must use the same timestamped body pose
  and the recorded static camera transform. Apparent synchronization does not prove
  geometric alignment; altitude datum and extrinsic errors move both products.
- RViz responsiveness and point size are visualization concerns. A cropped or
  decimated display cloud should be derived from, never substituted for, the
  full-resolution mapping and evaluation artifact.

## Baseline execution result

The complete bag-only run processed all 3,436 scans with no pose-gap rejection and
wrote 8,219,083 points at stride four. A matched single-header-pose control contains
the same ordered returns. Neither map accesses Bayern data.

The internal comparison sampled 2,054,771 matched returns. Per-point deskew changed
positions by 0.052 m median, 0.305 m at P90 and 0.597 m at P95; the 7.748 m maximum
occurs where angular motion and long range amplify pose interpolation. The robust
P90-P10 Z thickness in repeatedly occupied 0.25 m XY columns was 0.802 m median for
the deskewed map and 0.796 m for the control. Thus this first deskew baseline does
**not** demonstrate a mapping improvement and must not replace the control map yet.
It indicates that trajectory timing/extrinsics and tightly coupled LIO are necessary.
The column statistic includes real vertical structures and is not an accuracy metric.

## Optimized reconstruction and surfel result

The first unconstrained KISS-ICP smoke test failed the grounded-start validation:
it accumulated 15.65 m of scan-wise motion in ten seconds while onboard odometry
accumulated only 0.267 m. Investigation found that the upstream ROS adapter removed
NaN XYZ rows without removing matching timestamp rows. A project-owned adapter now
applies one shared mask. Even after that correction, flat-ground ICP remained
underconstrained, motivating the odometry prior and innovation gate.

The validated grounded run accumulates 0.321 m and ends 0.018 m from its start. The
complete run processed all 3,436 scans, reconstructed 7,983,601 points, and ran at
22.3 scans/s. It rejected 2,157 LiDAR innovations above 0.20 m, so its trajectory is
conservatively odometry anchored.

Ten-centimetre probabilistic fusion produced 3,135,787 occupied surfels. Every cell
stores centroid, observation count and covariance; 358,139 cells with at least five
observations also store a valid normal. Their median normal-direction standard
deviation is 0.0127 m and P95 is 0.0220 m. Sparse cells remain available for small
object inspection rather than being discarded. Global loop/GNSS graph optimization
and final reconstruction remain the next phase before held-out Bayern evaluation.

The internal repeated-column proxy is 0.829 m median for the reconstructed map,
compared with 0.796 m for the original single-pose control. This statistic mixes
ground with genuine vertical and dynamic structure, but it supplies no evidence that
the current optimization sharpened the complete map. Likewise, the 0.0127 m surfel
spread measures only points grouped inside one 10 cm voxel; it is not global mapping
accuracy. The new artifacts are therefore research intermediates, not a promoted
replacement for the control map.

A separate supported-surface product uses 20 cm voxels and requires at least three
observations. It retains 484,843 surfels, including 342,096 cells with valid normals,
and is intended for cleaner visual inspection. This product can suppress isolated
returns and merge nearby layers, but it can also remove sparsely observed small or
transient objects.

## Final bag-only vertical optimization

Controlled evidence localized the dominant defect to trajectory height rather than
raw LiDAR precision. Across the first 100 grounded scans, individual plane RMSE has
a `0.0116 m` median, fitted-plane offsets span only `0.0410 m`, and normals span
`0.109 deg`. During flight, however, held-out ground bias correlates `0.728` with
onboard altitude and reaches approximately `1.44 m`. The return to the launch area
provides an independent LiDAR loop closure of approximately `0.417 m` vertically.

The production correction is bag-only. Raw GNSS relative altitude establishes a
vertical scale of `0.94036`; endpoint GICP distributes the measured loop residual;
then a robust sparse bundle jointly estimates per-scan vertical offsets and terrain
heights from 3,141,850 low-surface observations in 45,991 cells seen by at least four
scans. Temporal second-difference regularization and the first/last 100 grounded
scans constrain the solution. No Bayern point enters this optimization.

After freezing the trajectory, scan-indexed held-out evaluation registers horizontal
coordinates using onboard GNSS only and uses Bayern solely for ground residuals. For
all 3,436 scans, vertical bias RMSE is `0.0583 m`, NMAD is `0.0313 m`, and P95
absolute bias is `0.1271 m`. The preceding loop-only result was `0.4036 m` RMSE and
`0.3452 m` NMAD. Median within-scan ground NMAD remains `0.0951 m`, which captures
beam noise, vegetation, correspondence error, and residual roll/pitch/extrinsic error.

The final direct reconstruction contains 8,006,176 points. Its recommended smooth
inspection product uses 10 cm cells with at least two observations and contains
948,195 surfels; median support is four returns, and covariance-derived normal spread
has a `0.0161 m` median and `0.0241 m` P95. This fusion follows trajectory correction
and does not conceal registration errors. The direct reconstruction remains the
master artifact so boxes, the person, the car, and other sparse changes are retained.

This result supports a reliable decimetre-class relative surface map for this bag; it
does not establish survey-grade absolute accuracy. The remaining `0.095 m` within-scan
ground spread cannot be safely removed by vertical strip fitting. A defensible next
improvement would require calibrated roll/pitch and LiDAR-body extrinsics with a
higher-rate IMU or surveyed control, followed by point-to-plane evaluation.

## Technical references

- Xu et al., [FAST-LIO2: Fast Direct LiDAR-Inertial Odometry](https://arxiv.org/abs/2107.06829).
- Shan et al., [LIO-SAM: Tightly-coupled LiDAR Inertial Odometry via Smoothing and Mapping](https://arxiv.org/abs/2007.00258).
- Lv et al., [Targetless Calibration of LiDAR-IMU System Based on Continuous-time Batch Estimation](https://april.zju.edu.cn/wp-content/papercite-data/pdf/lv2020targetlessco.pdf).
- Autoware Foundation, [Distortion Corrector](https://autowarefoundation.github.io/autoware_universe/pr-10077/sensing/autoware_pointcloud_preprocessor/docs/distortion-corrector/).
