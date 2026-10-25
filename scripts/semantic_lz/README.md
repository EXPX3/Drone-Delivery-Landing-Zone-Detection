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
above the Bayern terrain model. It rejects oversized masks, broad components, and
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

