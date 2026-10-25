# Dynamic Semantic Obstacle Replay

This branch adds synchronized open-vocabulary image segmentation and transfers
the image masks onto the corrected LiDAR scan. RViz keeps showing the complete
static and updated maps; semantic points are an additional RGB layer:

- magenta: person
- yellow: cargo box/package/crate
- blue: vehicle

The white-topped test box uses the appearance-specific prompt `white rectangular
obstacle`. On a captured missed frame this raised the correct detection from
0.0146 to 0.3586 without lowering the global 0.05 inference threshold.

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

The semantic RViz layer renders classified LiDAR returns as 0.22 m spheres.
Returns within 0.75 m are grouped into an object observation and associated with
same-class tracks within 1.20 m. Tracks require three observed frames and retain
only their latest compact point support plus a smoothed centroid, avoiding a
historical trail while tolerating adjacent-voxel projection jitter.

Object tracks require three independent frame observations before display. The
colored layer follows PointPainting and retains synchronized LiDAR returns inside
the image mask; those colors are semantic evidence, not geometric safety proof.
Safety accepts only returns 0.15-5.0 m above the Bayern terrain model. Camera projection uses the same strip-optimized
body trajectory as the deskewed LiDAR map; mixing raw camera poses with optimized
LiDAR poses previously caused repeated observations to spread into a halo.

Semantics and safety evidence are separate. Vision only identifies a region for
focused LiDAR verification. High-confidence vision requires at least two physical
above-ground LiDAR returns in one frame before those LiDAR voxels may block a
candidate. Image confidence alone never enters occupancy, so
a paper or painted marking misclassified as a box cannot reject a landing zone.
The ordinary LiDAR occupancy pipeline remains authoritative everywhere.

Generated weights and experiment images live under
`pcds/semantic_experiment/` and remain outside Git.

