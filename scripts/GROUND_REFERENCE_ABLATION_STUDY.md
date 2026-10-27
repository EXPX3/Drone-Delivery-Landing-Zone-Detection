# Semantic Ground-Reference Ablation

## Question

Does a detection-specific plane fitted from the live LiDAR scan provide better
physical support filtering than elevation relative to the static Bayern ground cloud?

## Controlled experiment

Both full-bag runs used the same bag, optimized trajectory, semantic model, image
frames, thresholds, and 12 Hz display sampling. Only the ground-reference tag changed.

- `bayern_ground_reference`: median of up to 12 classified Bayern ground points
  within 4 m of every projected LiDAR return.
- `local_detection_ground_reference`: deterministic 96-iteration RANSAC plane over
  current-scan returns within 5 m of each detection center; 0.15 m inlier threshold,
  at least 20 inliers, and at most 20 degrees slope. The plane is refined by least
  squares over its inliers.
- Both modes retain only returns 0.20-5.0 m above their selected ground reference.
- The local mode deliberately has no Bayern fallback.

Inputs were identical: 768 cargo-box, 133 person, and 27 vehicle image detections;
151,462 cargo, 788 person, and 1,043 vehicle projected mask-associated LiDAR returns.

## Results

### Detection population

The unit below is one image detection, not one physical object. Both matched runs
produced exactly the same image detections:

| Category | Image detections | LiDAR-evaluable detections | No usable synchronized evaluation |
|---|---:|---:|---:|
| Cargo box | 768 | 701 | 67 |
| Person | 133 | 117 | 16 |
| Vehicle | 27 | 20 | 7 |
| **Total** | **928** | **838** | **90** |

The 90 unevaluable detections are reported separately because the existing audit did
not retain their individual confidence and mask records. Assigning them afterward to
confidence, projection, or elevation failure would be artificial. All following
funnel percentages use the 838 detections for which every compared stage was recorded.

These counts are larger than the older 465-detection diagnostic in
`SEMANTIC_OBJECT_LOCALIZATION_RESULTS.md` because the detector prompts and thresholds
were subsequently expanded. The matched Bayern/local comparison is internally
controlled; the old and new detector populations must not be mixed.

### Cumulative detection funnel

#### Bayern ground reference

| Category | Evaluated | Confidence pass | Mask pass | Projected LiDAR | Elevated support | Compact 3D success |
|---|---:|---:|---:|---:|---:|---:|
| Cargo box | 701 | 599 (85.4%) | 599 (85.4%) | 514 (73.3%) | 170 (24.3%) | 170 (24.3%) |
| Person | 117 | 26 (22.2%) | 26 (22.2%) | 16 (13.7%) | 5 (4.3%) | 5 (4.3%) |
| Vehicle | 20 | 15 (75.0%) | 15 (75.0%) | 11 (55.0%) | 10 (50.0%) | 10 (50.0%) |
| **Total** | **838** | **640 (76.4%)** | **640 (76.4%)** | **541 (64.6%)** | **185 (22.1%)** | **185 (22.1%)** |

#### Detection-local ground reference

| Category | Evaluated | Confidence pass | Mask pass | Projected LiDAR | Elevated support | Compact 3D success |
|---|---:|---:|---:|---:|---:|---:|
| Cargo box | 701 | 599 (85.4%) | 599 (85.4%) | 514 (73.3%) | 160 (22.8%) | 160 (22.8%) |
| Person | 117 | 26 (22.2%) | 26 (22.2%) | 16 (13.7%) | 5 (4.3%) | 5 (4.3%) |
| Vehicle | 20 | 15 (75.0%) | 15 (75.0%) | 11 (55.0%) | 4 (20.0%) | 4 (20.0%) |
| **Total** | **838** | **640 (76.4%)** | **640 (76.4%)** | **541 (64.6%)** | **169 (20.2%)** | **169 (20.2%)** |

Here, “compact 3D success” counts successful image detections. A single successful
detection can contain more than one disconnected compact component, so the later
tracker receives 170/5/15 Bayern components and 161/5/5 local components for
cargo/person/vehicle respectively. Component counts must not be substituted for
detection-success counts in the funnel.

### Mutually exclusive first failure

Each of the 838 LiDAR-evaluable detections appears exactly once at its earliest
failing stage.

#### Bayern ground reference

| Category | Low confidence | Oversized mask | No projected LiDAR | Insufficient elevated support | Noncompact support | Success |
|---|---:|---:|---:|---:|---:|---:|
| Cargo box | 102 | 0 | 85 | 344 | 0 | 170 |
| Person | 91 | 0 | 10 | 11 | 0 | 5 |
| Vehicle | 5 | 0 | 4 | 1 | 0 | 10 |
| **Total** | **198 (23.6%)** | **0** | **99 (11.8%)** | **356 (42.5%)** | **0** | **185 (22.1%)** |

#### Detection-local ground reference

| Category | Low confidence | Oversized mask | No projected LiDAR | Insufficient elevated support | Noncompact support | Success |
|---|---:|---:|---:|---:|---:|---:|
| Cargo box | 102 | 0 | 85 | 354 | 0 | 160 |
| Person | 91 | 0 | 10 | 11 | 0 | 5 |
| Vehicle | 5 | 0 | 4 | 7 | 0 | 4 |
| **Total** | **198 (23.6%)** | **0** | **99 (11.8%)** | **372 (44.4%)** | **0** | **169 (20.2%)** |

Changing only the ground reference moves 16 detections from success to insufficient
elevated support: 10 cargo and 6 vehicle detections. Person results are identical.
This is the direct detection-level effect of the local plane in this experiment.

### Overlapping diagnostic failures

These counts are intentionally non-exclusive and must not be summed. They expose
conditions hidden behind an earlier first failure.

| Mode | Category | Low confidence | Oversized mask | No projected LiDAR | Insufficient elevated support |
|---|---|---:|---:|---:|---:|
| Bayern | Cargo box | 102 | 44 | 101 | 524 |
| Bayern | Person | 91 | 0 | 52 | 100 |
| Bayern | Vehicle | 5 | 0 | 4 | 5 |
| Local | Cargo box | 102 | 44 | 101 | 539 |
| Local | Person | 91 | 0 | 52 | 100 |
| Local | Vehicle | 5 | 0 | 4 | 14 |

### Observation-to-track outcome

| Mode | Class | Accepted compact components | Provisional tracks | Confirmed tracks |
|---|---|---:|---:|---:|
| Bayern | Cargo box | 170 | 17 | 6 |
| Bayern | Person | 5 | 1 | 1 |
| Bayern | Vehicle | 15 | 9 | 1 |
| Local | Cargo box | 161 | 8 | 5 |
| Local | Person | 5 | 1 | 1 |
| Local | Vehicle | 5 | 4 | 0 |

Only three cargo boxes, one person, and one vehicle are known scene objects. Therefore,
confirmed-track counts above those scene counts indicate duplicate localization or
false tracks, not additional objects. The local plane reduces cargo fragmentation but
does not eliminate it, and its vehicle configuration is too restrictive.

| Metric | Bayern | Detection-local | Change |
|---|---:|---:|---:|
| Elevated cargo points | 7,081 | 1,822 | -74.3% |
| Elevated person points | 101 | 101 | 0.0% |
| Elevated vehicle points | 1,037 | 155 | -85.1% |
| Compact accepted cargo points | 1,908 | 1,770 | -7.2% |
| Compact accepted person points | 44 | 44 | 0.0% |
| Compact accepted vehicle points | 594 | 70 | -88.2% |
| Confirmed cargo tracks | 6 | 5 | -1 |
| Confirmed person tracks | 1 | 1 | unchanged |
| Confirmed vehicle tracks | 1 | 0 | -1 |

The local method confirmed the three expected cargo locations, but still split repeated
observations into two additional cargo tracks. The Bayern method produced three
additional cargo tracks. Track count is not a calibrated precision metric, but the
reduction is consistent with less terrain leakage. Person localization was unchanged.
The loss of vehicle confirmation shows that one shared local-plane configuration is
not yet safe to adopt for every object scale.

## Plane quality and cost

There were 681 successful local fits. Empty/nonprojecting detections account for the
remaining fit records; no successful semantic support is inferred when fitting fails.

| Class | Successful fits | Median fit | P95 fit | Median plane residual P95 | Median slope |
|---|---:|---:|---:|---:|---:|
| Cargo box | 600 | 11.53 ms | 14.77 ms | 0.024 m | 0.91 deg |
| Person | 65 | 9.13 ms | 12.11 ms | 0.023 m | 0.45 deg |
| Vehicle | 16 | 6.65 ms | 9.93 ms | 0.047 m | 2.30 deg |

Because inference runs every third displayed frame and many frames contain no accepted
detection, the 681 fits over 3,439 displayed frames correspond to approximately
2.2 ms of measured plane fitting per displayed frame on average. Plane fitting is
therefore not the dominant cost relative to GPU segmentation and occupancy updates.

## Interpretation

The experiment supports using detection-local ground for small cargo/person evidence:
it removes broad ground leakage without reducing person evidence and only slightly
reduces compact cargo support. It does **not** establish higher end-to-end accuracy,
because the bag lacks exhaustive frame-level 3D ground-truth labels. The current local
fit also suppresses too much vehicle support and does not solve cross-pass association.

Keep `bayern_ground_reference` as the operational default while evaluating the local
tag. A production hybrid should use local ground for cargo/person, a larger
vehicle-specific neighborhood/model, and explicit fit-quality gating. It should not
silently substitute static-map height when live-ground fitting fails.

## Reproducible outputs

- `pcds/semantic_experiment/bayern_ground_reference_audit.json`
- `pcds/semantic_experiment/local_detection_ground_reference_audit.json`

These generated audit files are intentionally ignored by Git.
