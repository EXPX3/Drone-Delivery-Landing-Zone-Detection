# Dynamic Semantic Obstacle Replay

This branch adds synchronized open-vocabulary image segmentation and transfers
the image masks onto the corrected LiDAR scan. RViz keeps showing the complete
static and updated maps; semantic points are an additional RGB layer:

- magenta: person
- yellow: cargo box/package/crate
- blue: vehicle

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

The semantic RViz layer renders classified LiDAR returns as 0.22 m spheres and
stores them in a 0.20 m world-coordinate voxel memory. Missing detections never
erase a label. A voxel changes color only after two positive, contradictory
classifications at that location; this avoids flicker when a person or box is
temporarily outside the image or missed by the detector.

Semantic voxels require three independent frame observations before display;
duplicate returns from one scan count only once. Points below 0.12 m or above
3.0 m relative to the Bayern terrain model are rejected, removing ground returns
that project through an object silhouette. Camera projection uses the same strip-optimized
body trajectory as the deskewed LiDAR map; mixing raw camera poses with optimized
LiDAR poses previously caused repeated observations to spread into a halo.

After temporal and geometric confirmation, semantic person/box/vehicle voxels
are also persistent landing-safety evidence. A candidate whose footprint contains
one is blocked immediately; a candidate within the configured 2 m GNSS/wind
margin is downgraded to risky. Image recognition therefore supplements sparse
LiDAR occupancy but never makes a geometrically blocked candidate safer.

Generated weights and experiment images live under
`pcds/semantic_experiment/` and remain outside Git.

