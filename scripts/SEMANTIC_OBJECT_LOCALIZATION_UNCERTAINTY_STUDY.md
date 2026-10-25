# Semantic Object Localization and Uncertainty Study

## Objective

Determine whether repeated yellow box traces arise in the physical LiDAR map or
in camera-to-LiDAR semantic projection, then retain only defensible obstacle
tracks for live landing-zone revalidation.

## Method

The complete 346.7 s bag was replayed at 6 sampled image frames/s while retaining
every LiDAR return. Each 2D detection recorded confidence, mask area, LiDAR age,
UAV yaw, all mask-painted returns, terrain-height-verified returns, and nearby raw
elevated LiDAR. A detection was accepted only when image confidence, mask area,
point count, connected-component compactness, and height above the Bayern ground
surface all passed. Empirical covariance is calculated from independent 3D
observation centroids; it is not presented as calibrated Gaussian uncertainty.

## Diagnosis

| Quantity | Cargo-box result |
|---|---:|
| Image detections | 390 |
| Maximum mask area | 65.3% of image |
| Maximum mask-painted returns | 4,122 |
| Maximum mask-painted radial P95 | 7.57 m |
| Median height-verified radial P95 | 0.29 m |
| Median nearby raw-LiDAR radial P95 | 0.46 m |
| Semantic-to-raw center delta, median / P95 | 0.16 / 0.44 m |

The large rectangular yellow swaths are therefore projection-mask leakage: a
large 2D mask selected ground scan lines. They are not the shape of a physical
box in the LiDAR cloud. Three repeatable box tracks remain after physical gating:

| Track | Observations | SD X / Y | Radial P95 |
|---|---:|---:|---:|
| Box A | 31 | 0.19 / 0.13 m | 0.42 m |
| Box B | 26 | 0.25 / 0.24 m | 0.44 m |
| Box C | 20 | 0.26 / 0.11 m | 0.45 m |

An unstable fourth group had 0.64 / 0.53 m SD and 1.40 m radial P95. It is rejected
instead of being hidden by a larger association radius. Residual 0.4-0.5 m
cross-pass error plausibly combines provisional camera-LiDAR extrinsics, timing,
segmentation boundary error, and corrected-pose error. The current data does not
identify a unique causal split, so P95 is exposed to downstream consumers.

## Implemented Policy

- LiDAR geometry has final authority for landing safety.
- Vision proposes a class and image region; it cannot block a zone without
  elevated physical LiDAR support.
- Pathological masks and spatially broad components are rejected.
- Tracks keep only current compact support and bounded centroid history.
- Same-class instances get distinct colors, IDs, SD, and radial P95 labels.
- `/ddlzd/semantic_object_uncertainty` publishes machine-readable track state.
- Stale semantic evidence expires; ordinary LiDAR can still block immediately.
- The full map remains displayable, while safety processing stays local.

The thresholds are dataset-derived operating points, not universal constants.
Open-vocabulary scores are not calibrated probabilities; class-specific image
thresholds are therefore combined with deterministic 3D tests.

## Calibration Next Step

For metric claims below the measured 0.42-0.45 m P95, collect a dedicated target
sequence with hardware timestamps and broad roll/pitch/yaw excitation. Estimate
time offset and six-degree-of-freedom extrinsics jointly, then validate on held-out
traverses. Appropriate references include targetless mutual-information
calibration [Pandey et al.](https://onlinelibrary.wiley.com/doi/10.1002/rob.21542),
adaptive-voxel targetless calibration
[Liu et al.](https://hub.hku.hk/handle/10722/322266), and semantic mutual
information [Iyer et al.](https://arxiv.org/abs/2104.12023). Calibration should
reduce systematic projection error; empirical track uncertainty must still be
reported because segmentation and pose errors remain.

## Reproduction

The raw audit output is intentionally ignored as generated data:
`pcds/semantic_experiment/full_spread_audit.json`. Run the normal semantic replay
using `scripts/semantic_lz/run_semantic_replay.sh`; inspect the point layer,
uncertainty markers, camera overlay, and landing-zone updates together in RViz.
