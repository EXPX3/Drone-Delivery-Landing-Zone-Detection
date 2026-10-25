# Semantic-LiDAR Dynamic Landing-Zone Ablation Study

## Purpose

This experiment diagnoses sparse or spreading semantic colors and delayed
landing-zone rejection in the recorded UAV sequence. The required safety rule
is asymmetric: LiDAR geometry has final authority, while camera semantics may
support a decision only when a physical LiDAR return is also present.

## Implemented pipeline

1. Deskew each LiDAR scan and replay the strip-optimized UAV pose.
2. Transform every return into the registered Bayern map frame.
3. Process all returns within 25 m of the UAV for deterministic obstacle and
   landing-zone checks; retain a 3:1 sample only for map visualization/fusion.
4. Reject a landing zone immediately when one scan contains a range-adaptive
   physical cluster 0.15-5.0 m above its surface.
5. Run open-vocabulary instance segmentation on synchronized camera frames and
   project the full-resolution LiDAR scan into each mask (PointPainting).
6. Associate same-class observations over time for a stable colored semantic
   layer. Preserve the latest measured shape, rather than accumulating a trail.
7. Permit semantic support to affect safety only for physically elevated
   LiDAR points that also meet class confidence and cluster support thresholds.

This follows the projection-level fusion principle of [PointPainting](https://openaccess.thecvf.com/content_CVPR_2020/html/Vora_PointPainting_Sequential_Fusion_for_3D_Object_Detection_CVPR_2020_paper.html), while preserving an independent geometric path in the spirit of modular autonomous-driving perception stacks. LiDAR landing assessment uses the established geometric quantities of elevation, slope, roughness, and clearance described in [real-time airborne-LiDAR landing-zone identification](https://pmc.ncbi.nlm.nih.gov/articles/PMC10099213/). More tightly coupled BEV fusion such as [BEVFusion](https://papers.nips.cc/paper_files/paper/2022/hash/43d2b7fbee8431f7cef0d0afed51c691-Abstract-Conference.html) is a future option only after a labeled, calibrated UAV dataset exists.

## Controlled ablations

All rows replay the same first 110 s at 3 visualization FPS. Counts are direct
pipeline instrumentation, not manually inferred from RViz.

| Variant | LiDAR stride | Radius | Geometric confirmation | Painted semantic points | Elevated semantic points | Confirmed display tracks | Blocked LZs |
|---|---:|---:|---:|---:|---:|---:|---:|
| Previous baseline | 3 | 10 m | 3 scans | 124 | 2 | 0 | 31 |
| Height gate disabled | 3 | 10 m | 3 scans | 124 | 124 | 1 box | 31 |
| Full LiDAR, old display gate | 1 | 25 m | 1 scan | 406 | 7 | 0 | 176 |
| Selected pipeline | 1 | 25 m | 1 scan | 406 | 7 | 2 (person, box) | 176 |

The selected run processed 1,090 LiDAR scans and 312 image frames. It detected
8 person and 16 cargo-box instances, yielding 50 person-painted and 356
box-painted returns. Its confirmed tracks retained 8 person points and 23 box
points in the latest observation. Late-run LiDAR processing was approximately
40-65 ms per scan, compared with roughly 85 ms mean before map/safety sampling
was separated.

## Diagnosis

- **Every-third-point sampling was the main semantic sparsity cause.** Removing
  it increased projected support 3.27x (124 to 406 points).
- **The height filter removed 98.3% of painted points.** This is not evidence
  that those points belong to the obstacle: most are terrain returns that lie
  behind or inside a 2D mask. Coloring them is useful semantic evidence, but
  treating them as occupied height would generate false rejections.
- **Per-voxel temporal confirmation delayed safety.** One-scan, range-adaptive
  cluster confirmation increased rejected candidates 5.68x (31 to 176).
- **The white-topped box remains viewpoint dependent.** Open-vocabulary masks
  occur in only part of the sequence, and sparse LiDAR rays may strike its top
  or the ground visible inside the mask. Object-level tracking now preserves
  those detections, but cannot manufacture missing physical returns.
- **Accumulating projected points caused spreading.** Pose uncertainty and mask
  boundary variation displaced repeated observations. Keeping only the latest
  object support plus a smoothed centroid removes the historical trail.
- **Vision-guided safety produced no confirmed track in this segment.** Only
  seven painted returns passed the physical height test, and none simultaneously
  met the configured confidence and two-point per-frame support. Lowering these
  safeguards would increase recall but would also allow paper or mask leakage on
  grass to block valid landing areas.

## Selected settings

```yaml
dynamic_occupancy:
  obstacle_min_height_m: 0.15
  vertical_clearance_m: 5.0
  confirmation_hits: 1
  rolling_horizontal_radius_m: 25.0
  point_stride: 1
  map_point_stride: 3
  zone_publish_hz: 10.0

semantic_segmentation:
  minimum_height_above_ground_m: 0.15
  maximum_height_above_ground_m: 5.0
  vision_guided_lidar:
    minimum_lidar_points_per_frame: 2
    confirmation_frames: 1
```

The ordinary and semantic physical gates now share the same 0.15-5.0 m range.
They intentionally retain different reference surfaces: a candidate's own
surface for deterministic landing clearance and the registered terrain model
for rejecting camera-mask leakage before semantic association.

## Limitations and next experiment

There are no frame-level ground-truth masks or 3D object boxes in this bag, so
precision, recall, mAP, or IoU cannot be claimed. The quantitative results above
measure pipeline behavior only. Research-grade semantic accuracy requires a
labeled subset containing every box, person, car, grass, and paper instance,
followed by calibration-error sweeps and held-out evaluation. The current
implementation deliberately prefers a missed semantic color over allowing
vision-only evidence to override contradictory LiDAR geometry.
