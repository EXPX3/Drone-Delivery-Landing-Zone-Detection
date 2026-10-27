# Dynamic Semantic Obstacle Replay

This branch adds synchronized open-vocabulary image segmentation and transfers
the image masks onto the corrected LiDAR scan. RViz keeps showing the complete
static and updated maps; compact physically verified semantic tracks are an
additional RGB layer:

- magenta: person
- bright yellow (`RGB 255,255,0`): cargo box/package/crate
- blue: vehicle

The white-topped test box uses the appearance-specific prompt `white rectangular
obstacle`. On a captured missed frame this raised the correct detection from
0.0146 to 0.3586 without lowering the global 0.05 inference threshold.
The black test box similarly uses `black rectangular obstacle`, which raised its
verified missed-frame score above threshold to 0.1056. Semantic safety still
requires synchronized elevated LiDAR returns; an image detection alone cannot
reject a candidate.

Prepare the reproducible GPU image and model files once:

```bash
cd /home/gvb/Documents/robotspace/DDLZD_ws/Drone-Delivery-Landing-Zone-Detection
scripts/semantic_lz/prepare_models.sh
```

Run the experiment:

```bash
docker rm -f ddlzd-lz-rviz 2>/dev/null || true
scripts/semantic_lz/run_semantic_replay.sh \
  /media/gvb/ssd24ubuntu2/bags/Bagfilles_Feb13_2026/recovered_bag \
  1.0 12
```

The semantic model runs every third displayed frame at 1280 pixels. Mapping and
the geometry-based safety rejection remain independent: an unrecognized obstacle
can still invalidate a landing zone. The semantic layer is therefore descriptive
evidence, not a replacement for geometric collision checking.

The semantic RViz layer renders only compact connected LiDAR components 0.20-5.0 m
above a selectable ground reference. It rejects oversized masks, broad components, and
class-specific low-confidence detections. Same-class instances receive distinct
track IDs and labels with the full empirical XY covariance matrix and radial P95.
RViz draws oriented 1/2/3-sigma contours plus a normalized Gaussian wireframe;
the wireframe height represents relative likelihood, not physical altitude. Tracks retain
only current physical support and a bounded observation history. Once confirmed by
three frames they remain visible for cross-pass comparison. Only support observed
within the latest 90 displayed frames refreshes landing-zone hazards, so persistent
visualization does not make a stale position permanently unsafe.

The former display colored every return inside a mask. A full-bag audit found masks
covering 65.3% of the image and painting 4,122 returns with 7.57 m radial spread;
those were ground scan lines, not box geometry. The corrected three box tracks have
0.42-0.45 m radial P95 across traverses. See
`scripts/SEMANTIC_OBJECT_LOCALIZATION_UNCERTAINTY_STUDY.md`.

Camera projection uses the same strip-optimized
body trajectory as the deskewed LiDAR map; mixing raw camera poses with optimized
LiDAR poses previously caused repeated observations to spread into a halo.

Semantics and safety evidence are separate. Vision only identifies a region for
focused LiDAR verification. High-confidence vision requires at least two physical
above-ground LiDAR returns in one frame before those LiDAR voxels may block a
candidate. Image confidence alone never enters occupancy, so
a paper or painted marking misclassified as a box cannot reject a landing zone.
The ordinary LiDAR occupancy pipeline remains authoritative everywhere. Current
track state is also published as JSON on `/ddlzd/semantic_object_uncertainty` for
downstream consumers.

Confirmed LiDAR hazards are monotonic during a replay: once physical returns block
a landing footprint, that footprint remains blocked for the run. Automatic decay is
disabled because endpoint-only scans do not provide sufficient ray-miss evidence to
prove that a box has disappeared. A future dynamic-clearing mode must use explicit
ray-based free-space integration rather than absence of repeated endpoints.

## Ground-reference tags

The existing behavior remains the default:

```yaml
ground_reference_mode: bayern_ground_reference
```

The detection-local alternative fits a robust near-horizontal plane to the current
registered LiDAR scan within 5 m of each image detection:

```yaml
ground_reference_mode: local_detection_ground_reference
```

It can also be selected without modifying YAML:

```bash
python3 scripts/lz_rviz_viewer/replay_node.py \
  --bag /bag \
  --semantic-ground-reference local_detection_ground_reference
```

The local method does not silently fall back to Bayern when a plane cannot be fitted.
This keeps comparisons between the two tags interpretable. Full matched-run results,
including runtime and failure modes, are in
`scripts/GROUND_REFERENCE_ABLATION_STUDY.md`.

Generated weights and experiment images live under
`pcds/semantic_experiment/` and remain outside Git.

Render the persistent orthophoto summary after a metrics replay:

```bash
/home/gvb/Documents/robotspace/venvs/praktikum_flightplanning_venv/bin/python \
  scripts/semantic_lz/render_object_orthophoto.py
```

The output `pcds/semantic_experiment/confirmed_objects_orthophoto_metrics.png`
contains confirmed object positions, all pairwise relative offsets and distances,
observation counts, radial P95, covariance matrices, and 95% XY covariance ellipses.
It also overlays the supplied relative ground truth after the documented rigid UTM
registration: cyan crosses and residual arrows compare it with the retained detection
centers, while a white square and blue triangle mark the inferred survey origin and
the independently recovered UAV start.

Generate the uncluttered survey-coordinate variant without changing the original:

```bash
docker run --rm -i -v "$PWD:/workspace" -w /workspace \
  --entrypoint python3 ddlzd-semantic:2026 \
  scripts/semantic_lz/render_object_orthophoto.py \
  --ground-truth-axes-layout \
  --output pcds/semantic_experiment/confirmed_objects_ground_truth_axes.png
```

This version retains only the white inter-object connecting lines, uses ROS-axis
colors (`+X` red, `+Y` green), projects each point orthogonally onto both axes, and
places the actual local `x` and `y` distances in metres at conventional axis ticks
(X labels below the X-axis and Y labels outside the Y-axis). Its ground-truth residuals are also
reported in that local frame, while covariance remains explicitly labelled in the
East/North map frame. It uses dark-grey 95% covariance contours and smaller detection centers. The original metrics image
remains unchanged.

Render the full-bag 2D/3D covariance-evolution video:

```bash
/home/gvb/Documents/robotspace/venvs/praktikum_flightplanning_venv/bin/python \
  scripts/semantic_lz/render_covariance_evolution_video.py
```

The 34.8 s MP4 replays the 346.7 s bag at 10x. Its orthophoto panel remains
metric; the 3D Gaussian footprints use a labeled 4x horizontal display scale so
sub-metre covariance changes remain visible against the multi-object scene. The
georeferenced ground-truth XY layout remains visible at low opacity for the complete
video. In the 3D panel it is drawn only at the `z=0` reference plane and labeled
`XY only`; no ground-truth elevation is asserted. Predicted physical Z remains in
the live metrics, while the 3D vertical axis continues to represent normalized
Gaussian density rather than elevation.

