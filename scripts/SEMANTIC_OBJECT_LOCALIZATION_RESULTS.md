# Full-Bag Semantic Object Localization Results

## Scope

This report aggregates every accepted camera-associated LiDAR observation from the
346.7 s bag for the three validated cargo boxes and one person. It reports empirical
cross-observation repeatability, not absolute ground-truth accuracy.

An observation was accepted only when it passed the deployed image-confidence,
mask-area, elevated-point-count, terrain-height, and connected-component compactness
tests. Accepted observations were associated within 1.2 m of the four validated
tracks. Later isolated detections and the fourth unvalidated cargo candidate were
excluded rather than being forced into a known object.

## Aggregate Results

| Object | Accepted observations | Observation interval [s] | Median confidence | Robust UTM position E, N, Z [m] | Radial P95 [m] |
|---|---:|---:|---:|---|---:|
| Cargo 1 | 25 | 51.26-216.94 | 0.818 | 696499.013, 5328142.584, 549.761 | 0.438 |
| Cargo 2 | 20 | 56.76-221.96 | 0.732 | 696507.208, 5328145.435, 549.635 | 0.452 |
| Cargo 3 | 31 | 118.90-273.45 | 0.331 | 696493.389, 5328140.570, 549.925 | 0.421 |
| Person 1 | 3 | 129.46-131.08 | 0.538 | 696501.053, 5328137.188, 550.754 | 0.220 |

Cargo 1/2/3 accumulated 248/190/429 elevated semantic LiDAR returns across their
accepted observations. Person 1 accumulated 14 returns. The cargo-box result is
therefore supported across repeated traverses; the person result is preliminary
because only three temporally adjacent observations passed all gates.

## Full Detection Funnel

The unit in this section is one image detection, not one unique physical object.
The complete bag produced 465 detections: 390 cargo-box, 65 person, and 10 vehicle
detections. A pipeline failure means that a detection could not become a trusted
synchronized 3D observation; it does not necessarily mean the image detection was
semantically wrong.

### Cumulative Success

| Category | Image detections | Confidence pass | Mask-size pass | Projected LiDAR present | Elevated support pass | Final compact 3D success |
|---|---:|---:|---:|---:|---:|---:|
| Cargo box | 390 | 325 (83.3%) | 325 (83.3%) | 280 (71.8%) | 90 (23.1%) | 90 (23.1%) |
| Person | 65 | 18 (27.7%) | 18 (27.7%) | 11 (16.9%) | 3 (4.6%) | 3 (4.6%) |
| Vehicle | 10 | 8 (80.0%) | 8 (80.0%) | 6 (60.0%) | 6 (60.0%) | 6 (60.0%) |
| **Total** | **465** | **351 (75.5%)** | **351 (75.5%)** | **297 (63.9%)** | **99 (21.3%)** | **99 (21.3%)** |

The mask-size stage does not remove an additional confidence-passing detection.
The 32 pathological oversized cargo masks observed in the independent diagnostics
all already fail the camera-confidence gate. Keeping the explicit mask limit is
still necessary protection against a future confident oversized mask.

### Mutually Exclusive First Failure

Each detection is counted exactly once at its earliest failing stage:

| Category | Low camera confidence | Oversized mask | No projected LiDAR | Insufficient elevated LiDAR | Noncompact support | Success |
|---|---:|---:|---:|---:|---:|---:|
| Cargo box | 65 | 0 | 45 | 190 | 0 | 90 |
| Person | 47 | 0 | 7 | 8 | 0 | 3 |
| Vehicle | 2 | 0 | 2 | 0 | 0 | 6 |
| **Total** | **114** | **0** | **54** | **198** | **0** | **99** |

Thus, 24.5% of all detections fail first at camera confidence, 11.6% have no
projected LiDAR return after passing image gates, 42.6% lack sufficient elevated
physical support, and 21.3% become trusted 3D observations.

### Overlapping Diagnostic Failures

The following counts are intentionally non-exclusive and therefore must not be
summed. They show every failed condition regardless of earlier gates:

| Category | Low confidence | Oversized mask | No projected LiDAR | Insufficient elevated LiDAR | Noncompact support |
|---|---:|---:|---:|---:|---:|
| Cargo box | 65 | 32 | 54 | 293 | 5 |
| Person | 47 | 0 | 28 | 55 | 0 |
| Vehicle | 2 | 0 | 3 | 3 | 0 |

The person bottleneck is primarily multimodal overlap: 65 image detections reduce
to 37 with any projected LiDAR, 12 with any height-verified return, and only 3 with
the required confidence and at least two compact elevated returns. The camera saw
the person repeatedly, but the synchronized LiDAR evidence was usually absent or
too sparse for trusted 3D localization.

### Observation-to-Track Outcome

Spatial association of the 99 successful observations gives:

- Cargo: 90 successful observations form 12 provisional spatial tracks. Four reach
  the three-observation threshold; only three correspond to the validated physical
  cargo boxes. The fourth has five observations but remains unvalidated and is not
  included in the reported three-box result.
- Person: all 3 successful observations form one confirmed track.
- Vehicle: 6 successful observations form four provisional tracks; one reaches
  three observations at the car location.

This distinction is essential: passing per-observation gates is necessary but does
not by itself prove that a resulting track corresponds to a known physical object.

## Covariance

The empirical horizontal covariance matrices are in square metres:

```text
Cargo 1: [[ 0.04013, -0.00172],   principal SD: 0.201, 0.131 m
          [-0.00172,  0.01731]]

Cargo 2: [[ 0.06936,  0.00687],   principal SD: 0.265, 0.109 m
          [ 0.00687,  0.01264]]

Cargo 3: [[ 0.03611,  0.00113],   principal SD: 0.190, 0.133 m
          [ 0.00113,  0.01777]]

Person 1: [[ 0.02168,  0.00091],  principal SD: 0.147, 0.034 m
           [ 0.00091,  0.00118]]
```

Cargo 2 has the largest major-axis uncertainty. Cargo 1 and Cargo 3 have similar,
mildly anisotropic repeatability. The person covariance appears smaller, but its
three-sample estimate is not sufficiently mature for a strong distributional claim.

## Image-Model Confidence

Confidence is stored for every image detection and propagated into the object
observations. It is the open-vocabulary segmenter's score for the image mask, not a
calibrated probability that the 3D object exists. A detection must also pass the
independent LiDAR support tests before it contributes to these statistics.

| Object | Acceptance threshold | Minimum | Mean | Median | Maximum |
|---|---:|---:|---:|---:|---:|
| Cargo 1 | 0.10 | 0.509 | 0.787 | 0.818 | 0.895 |
| Cargo 2 | 0.10 | 0.427 | 0.681 | 0.732 | 0.874 |
| Cargo 3 | 0.10 | 0.122 | 0.348 | 0.331 | 0.617 |
| Person 1 | 0.30 | 0.345 | 0.509 | 0.538 | 0.646 |

Cargo 1 and Cargo 2 have strong and consistent image evidence. Cargo 3 is supported
primarily by repeated compact elevated LiDAR agreement despite weaker image scores.
Person 1 exceeds its stricter 0.30 threshold in all three accepted observations,
but the sample count remains too small for a strong confidence-stability claim.

Confidence and covariance answer different questions: confidence measures image
recognition evidence for one observation, while covariance measures the spatial
spread of repeated accepted 3D localizations. They are therefore reported side by
side and are not multiplied into one uncalibrated score.

### Camera, LiDAR, and Combined Evidence

For separate modality reporting, the following bounded engineering scores are used:

```text
point support:       Cn = 1 - exp(-N / Nref)
compactness:         Cr = exp(-0.5 * (radial_P95 / radial_limit)^2)
LiDAR evidence:      Clidar = sqrt(Cn * Cr)
combined evidence:   Cfused = sqrt(Ccamera * Clidar)
```

`Nref` is 10 returns for cargo and 5 for a person. The radial limits are the
deployed compactness limits, 0.75 m and 0.80 m respectively. The geometric mean is
conservative: either weak modality lowers the fused score. Median full-bag results:

| Object | Camera-only median [range] | LiDAR-only median [range] | Combined median [range] |
|---|---:|---:|---:|
| Cargo 1 | 0.818 [0.509, 0.895] | 0.705 [0.425, 0.901] | 0.755 [0.465, 0.891] |
| Cargo 2 | 0.732 [0.427, 0.874] | 0.757 [0.462, 0.886] | 0.722 [0.466, 0.853] |
| Cargo 3 | 0.331 [0.122, 0.617] | 0.779 [0.426, 0.898] | 0.505 [0.310, 0.739] |
| Person 1 | 0.538 [0.345, 0.646] | 0.671 [0.574, 0.895] | 0.555 [0.481, 0.760] |

The separation reveals useful behavior hidden by one score: Cargo 3 has weak camera
evidence but the strongest median LiDAR evidence, while Cargo 1 is strongest in the
camera. The combined score does not permit strong LiDAR geometry to completely hide
weak visual recognition.

Only `Ccamera` is emitted by a learned detector. `Clidar` and `Cfused` are explicitly
defined, reproducible evidence scores, not calibrated probabilities. Converting
them to probabilities requires labeled positive/negative scenes and held-out
calibration using reliability diagrams, Brier score, or expected calibration error.

The full-bag robust centers moved 0.181, 0.306, and 0.367 m from the corresponding
150 s validation centers for Cargo 1/2/3. Person 1 moved 0.091 m. This demonstrates
why covariance must evolve as repeated passes arrive and why a first-pass location
should not be treated as final.

## Relative Geometry

Offsets are from the first object to the second, in UTM east/north metres:

| Pair | dE [m] | dN [m] | Distance [m] |
|---|---:|---:|---:|
| Cargo 1 -> Cargo 2 | +8.195 | +2.851 | 8.677 |
| Cargo 1 -> Cargo 3 | -5.624 | -2.014 | 5.974 |
| Cargo 1 -> Person 1 | +2.040 | -5.395 | 5.768 |
| Cargo 2 -> Cargo 3 | -13.819 | -4.865 | 14.650 |
| Cargo 2 -> Person 1 | -6.155 | -8.246 | 10.290 |
| Cargo 3 -> Person 1 | +7.664 | -3.381 | 8.377 |

## Interpretation

1. The three cargo boxes are spatially distinct and repeatedly observed. Their
   radial P95 values cluster tightly between 0.421 and 0.452 m.
2. Most giant yellow patterns seen before filtering were not cargo geometry. The
   earlier audit found masks covering up to 65.3% of an image and painting returns
   across 7.57 m; compact elevated support removed those failures.
3. The remaining cargo spread is a combined empirical effect of camera-LiDAR
   extrinsics, timing, segmentation boundaries, corrected UAV pose, and LiDAR
   sampling. These data alone cannot uniquely apportion those sources.
4. The person was localized consistently over 1.63 s, but only three accepted
   observations are available. More viewpoints are required before using its
   covariance as a calibrated uncertainty model.
5. These covariance matrices quantify repeatability. Absolute accuracy still
   requires surveyed object centers or a calibrated reference target.

## Artifacts

- Static orthophoto: `pcds/semantic_experiment/confirmed_objects_orthophoto_metrics.png`
- Full evolution video: `pcds/semantic_experiment/semantic_localization_covariance_evolution.mp4`
- Raw full-bag audit: `pcds/semantic_experiment/full_spread_audit.json`
- Reproducible video renderer: `scripts/semantic_lz/render_covariance_evolution_video.py`
