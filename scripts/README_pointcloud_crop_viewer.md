# LAZ Crop And Satellite Viewer

This folder contains scripts for cropping Bayern LAZ point clouds with a GPX polygon and visualizing the cropped result over an orthophoto base layer.

## Environment

Use the existing virtual environment:

```bash
/home/gvb/Documents/robotspace/venvs/praktikum_flightplanning_venv/bin/python3
```

Required Python packages are installed there:

```text
laspy
lazrs
numpy
pyproj
open3d
plotly
pillow
requests
```

## Crop LAZ With GPX

Run from the repository root:

```bash
cd /home/gvb/Documents/robotspace/DDLZD_ws/Drone-Delivery-Landing-Zone-Detection

/home/gvb/Documents/robotspace/venvs/praktikum_flightplanning_venv/bin/python3 \
scripts/crop_laz_with_gpx.py \
--laz pcds/696_5328.laz \
--gpx pcds/bayernatlas_20260812084517.gpx \
--epsg 25832
```

Outputs:

```text
pcds/696_5328_cropped_by_bayernatlas_20260812084517.pcd
```

The Bayern LAZ files do not contain CRS metadata in the header, so `--epsg 25832` is required. Bayern laser data is delivered in UTM zone 32N / EPSG:25832.

## Crop With Classification Field

The downloaded LAZ files include LAS classification codes. To preserve the class value as an extra PCD field:

```bash
/home/gvb/Documents/robotspace/venvs/praktikum_flightplanning_venv/bin/python3 \
scripts/crop_laz_with_gpx.py \
--laz pcds/696_5328.laz \
--gpx pcds/bayernatlas_20260812084517.gpx \
--epsg 25832 \
--include-classification \
--out-prefix 696_5328_cropped_by_bayernatlas_20260812084517_classified
```

Output:

```text
pcds/696_5328_cropped_by_bayernatlas_20260812084517_classified.pcd
```

Observed cropped classification counts:

```text
2  ground:         490412
20 ignored ground: 192868
```

Common relevant LAS classes in these Bayern tiles:

```text
2  ground
6  building
7  low point/noise
20 ignored ground
22 temporal exclusion
```

## Generate Satellite Overlay HTML

The GPX file only contains a polygon boundary. It does not contain a Google Maps 3D mesh. For a meaningful visual base layer, use the official Bayern DOP20 orthophoto WMS as a satellite/orthophoto layer.

Generate the viewer:

```bash
/home/gvb/Documents/robotspace/venvs/praktikum_flightplanning_venv/bin/python3 \
scripts/visualize_pcd_gpx_mesh.py \
--pcd pcds/696_5328_cropped_by_bayernatlas_20260812084517.pcd \
--html pcds/696_5328_cropped_by_bayernatlas_20260812084517_satellite_overlay.html \
--orthophoto pcds/696_5328_cropped_by_bayernatlas_20260812084517_satellite_overlay_dop20.png \
--html-max-points 180000 \
--orthophoto-stride 4 \
--no-gpx
```

Open the viewer directly in Chrome:

```bash
google-chrome file:///home/gvb/Documents/robotspace/DDLZD_ws/Drone-Delivery-Landing-Zone-Detection/pcds/696_5328_cropped_by_bayernatlas_20260812084517_satellite_overlay.html
```

Do not use `xdg-open` on this machine, because it may route the HTML file to Element instead of a browser.

## Fetch A Fresh Bayern DOP20 Orthophoto

If the orthophoto PNG is missing or you want to regenerate it:

```bash
/home/gvb/Documents/robotspace/venvs/praktikum_flightplanning_venv/bin/python3 \
scripts/visualize_pcd_gpx_mesh.py \
--pcd pcds/696_5328_cropped_by_bayernatlas_20260812084517.pcd \
--html pcds/696_5328_cropped_by_bayernatlas_20260812084517_satellite_overlay.html \
--fetch-bayern-dop20 \
--html-max-points 180000 \
--orthophoto-stride 4 \
--no-gpx
```

This writes:

```text
pcds/696_5328_cropped_by_bayernatlas_20260812084517_satellite_overlay_dop20.png
pcds/696_5328_cropped_by_bayernatlas_20260812084517_satellite_overlay.html
```

The WMS endpoint used by the script is:

```text
https://geoservices.bayern.de/od/wms/dop/v1/dop20
```

Layer:

```text
by_dop20c
```

## Visual Layer Controls

The HTML viewer uses Plotly. Use the legend to toggle layers on and off:

```text
Bayern DOP20 orthophoto
Point cloud
LZ 1: r=2.62 m
```

Current recommended viewer generation uses `--no-gpx`, so the GPX crop block is not highlighted.

If a GPX mesh is generated later and you want it included but hidden by default, omit `--no-gpx`. Then click the GPX legend entries in the viewer to show/hide them. Add `--show-gpx` if you want the GPX layer visible by default.

## Open3D Viewer Note

The Open3D native viewer may fail on Wayland with:

```text
Failed to initialize GLEW
```

Use the HTML viewer above for this machine. It avoids GLFW/GLEW and opens in a normal browser.

## Region-Growing Landing-Zone Detection

The generated full cropped PCD is configured as the input for the C++ `segmentPointCloud` / `Region_Growing_Segmentation` method. The final run uses all points in the cropped PCD and does not rely on LAS classification labels.

Config file:

```text
lib/config/algo_testing_own_pcd_config.yaml
```

Important current values:

```yaml
enable_visualization: false
run_detection_once: true
use_pcd_file: true
pcd_file_path: "/home/airsim_user/Drone-Delivery-Landing-Zone-Detection/pcds/696_5328_cropped_by_bayernatlas_20260812084517.pcd"
landing_zones_csv_path: "/home/airsim_user/Drone-Delivery-Landing-Zone-Detection/pcds/region_growing_landing_zones_max10000_radius2_5_to3.csv"
max_landingZones: 10000
active_tags:
  - USE_ALL_CLASSES_RADIUS_RANGE_EXPERIMENT

min_radius_threshold: 2.5
max_radius_threshold: 3.0
max_slope_threshold: 12
min_point_density_threshold: 8.0
max_relief_threshold: 5.0
max_roughness_threshold: 2.0
step_size: 0.75

algorithms:
  - name: Region_Growing_Segmentation
    curvature_threshold: 0.08
    min_cluster_size: 120
    alpha: 0.8
    cluster_tolerance: 0.85
```

Why these values:

```text
run_detection_once: true
  Runs the configured PCD once and skips the Monte Carlo parameter sweep.

enable_visualization: false
  Avoids PCL/OpenGL windows inside Docker. Visualization is done in browser HTML instead.

pcd_file_path: 696_5328_cropped_by_bayernatlas_20260812084517.pcd
  Uses the full cropped point cloud with all classes. A class-2-only run was used only as a diagnostic to prove the large flat ground was detectable.

min_radius_threshold: 2.5
  Keeps candidate landing zones at roughly 5.0 m diameter or larger.

max_radius_threshold: 3.0
  Caps accepted variable-radius landing zones at 3.0 m. Change this value in the config if you want a wider or narrower radius range.

max_slope_threshold: 12
  The accepted candidate has about 1.57 deg slope, so 12 deg is permissive enough for this terrain without accepting obviously tilted regions.

min_point_density_threshold: 8.0
  The cropped cloud has about 15.4 pts/m2 over the bounding box and the accepted zone has about 39.5 pts/m2.

max_relief_threshold: 5.0
  The accepted region reports relief around 4.26 m. Lower values rejected every candidate on this real cropped dataset.

max_roughness_threshold: 2.0
  The accepted region reports roughness around 1.69. Lower values rejected every candidate.

curvature_threshold: 0.08
  Slightly relaxed from the earlier 0.05 so region growing keeps connected ground-like surfaces on the real LAZ-derived cloud.

min_cluster_size: 120
  Filters tiny fragments while keeping small candidate regions.

alpha: 0.8
  Produces usable alpha-shape boundaries for the cropped cloud scale and sparse-ish local point spacing.

cluster_tolerance: 0.85
  Matches the sampled nearest-neighbor spacing, roughly 0.53 m median and 1.05 m p90, so nearby points connect without over-merging everything.

max_landingZones: 10000
  Increases the number of raw circle candidates considered. All other thresholds are unchanged from the previous full-cloud run.

active_tags: USE_ALL_CLASSES_RADIUS_RANGE_EXPERIMENT
  Runs detection directly on the all-classes cropped PCD and accepts candidate radii from min_radius_threshold through max_radius_threshold. The current configured run allows 2.5 m <= radius <= 3.0 m.

active_tags: USE_ALL_CLASSES_EXACT_RADIUS_2_5_ONLY_EXPERIMENT
  Runs detection directly on the all-classes cropped PCD, keeps candidate centers with at least 2.5 m clearance, then forces the exported/evaluated landing-zone radius to exactly 2.5 m. This is useful when you want many fixed-size landing-zone placements from an unclassified point cloud instead of ranking by largest possible radius.
```

Additional code tuning:

```text
circleFitting max_num_of_lz_per_cluster now follows max_landingZones.
```

The original code had a hidden hard cap of 5 landing zones per cluster. Since the large flat ground appears as one dominant region-growing cluster, that cap prevented `max_landingZones` from producing many candidate solutions. The cap was changed to use the configured maximum.

Tuning iterations performed:

```text
1. Original full PCD before the per-cluster cap fix:
   Found only 1 unique accepted LZ, visually in a bad/tree-adjacent area.

2. Class-2-only ground PCD, same parameters:
   Found 4 unique LZs on the large flat ground.

3. Class-2-only ground PCD plus fixed per-cluster LZ cap:
   Found 29 passing ranked circles and exported 19 unique landing zones after de-duplication.

4. Full all-point PCD plus fixed per-cluster LZ cap:
   Found 8 unique landing zones without using class labels.

5. Full all-point PCD with only max_landingZones increased to 200:
   Found 31 passing ranked circles and exported 20 unique landing zones after de-duplication. This result is preserved as pcds/region_growing_landing_zones_max200.csv.

6. Full all-point PCD with only max_landingZones increased to 2000:
   Found 554 passing ranked circles and exported 120 unique landing zones after de-duplication. The previous max-2000 CSV remains available as a comparison result.

7. Full all-point PCD with max_landingZones increased to 10000, variable radius:
   Found 2171 passing ranked circles and exported 175 unique landing zones after de-duplication.

8. Full all-point PCD with max_landingZones increased to 10000 and TAG: USE_ALL_CLASSES_EXACT_RADIUS_2_5_ONLY_EXPERIMENT:
   Found 5758 passing ranked circles and exported 1790 unique landing zones after de-duplication.

9. Full all-point PCD with max_landingZones increased to 10000 and active_tags: USE_ALL_CLASSES_RADIUS_RANGE_EXPERIMENT:
   Found 5382 passing ranked circles and exported 1356 unique landing zones after de-duplication. The current configured radius band is 2.5 m to 3.0 m, adjustable with min_radius_threshold and max_radius_threshold. Candidate centers with larger available clearance are kept and exported/evaluated at max_radius_threshold instead of being rejected.
```

Build and run in Docker:

```bash
cd /home/gvb/Documents/robotspace/DDLZD_ws/Drone-Delivery-Landing-Zone-Detection

docker run --name DDLZD-region-growing-run --rm --privileged --network host \
-v /home/gvb/Documents/robotspace/DDLZD_ws/Drone-Delivery-Landing-Zone-Detection:/home/airsim_user/Drone-Delivery-Landing-Zone-Detection \
--entrypoint /bin/bash \
giri6937/lam:latest \
-lc 'cd /home/airsim_user/Drone-Delivery-Landing-Zone-Detection/lib && mkdir -p build && cd build && cmake .. && make -j2 && ./main'
```

The detector writes:

```text
pcds/region_growing_landing_zones_max10000_radius2_5_to3.csv
```

Current result:

```csv
rank,x,y,z,radius,normal_x,normal_y,normal_z
1,696389.875043,5328191.042400,549.461305,3.000000,0.000393,0.001164,0.999999
2,696395.656204,5328106.531589,549.557362,3.000000,0.000393,0.001164,0.999999
3,696393.468754,5328129.989447,549.530929,3.000000,0.000393,0.001164,0.999999
...
1356,696455.497660,5328084.813777,549.570189,3.000000,0.000393,0.001164,0.999999
```

The detector produced 5382 passing ranked circles and exported 1356 unique radius-range landing zones from the full cropped point cloud. Tree/obstacle points are retained in the input and are handled by the vertical-collision and hazard checks. The CSV writer uses fixed decimal precision so UTM coordinates are not rounded into coarse/e-scientific notation. Radius-range mode caps large-clearance candidates at max_radius_threshold, so open interior areas are retained instead of only selecting edge-adjacent 2.5-3.0 m clearances.

Previous comparison CSVs remain available here:

```text
pcds/region_growing_landing_zones_max200.csv
pcds/region_growing_landing_zones_max2000.csv
pcds/region_growing_landing_zones_max10000.csv
pcds/region_growing_landing_zones_max10000_exact_radius2_5.csv
```

Regenerate the browser overlay with the detected solution:

```bash
/home/gvb/Documents/robotspace/venvs/praktikum_flightplanning_venv/bin/python3 \
scripts/visualize_pcd_gpx_mesh.py \
--pcd pcds/696_5328_cropped_by_bayernatlas_20260812084517.pcd \
--html pcds/696_5328_cropped_full_satellite_lz_overlay_max10000_radius2_5_to3.html \
--orthophoto pcds/696_5328_cropped_by_bayernatlas_20260812084517_satellite_overlay_dop20.png \
--landing-zones-csv pcds/region_growing_landing_zones_max10000_radius2_5_to3.csv \
--html-max-points 180000 \
--orthophoto-stride 4 \
--no-gpx
```

Open:

```bash
google-chrome file:///home/gvb/Documents/robotspace/DDLZD_ws/Drone-Delivery-Landing-Zone-Detection/pcds/696_5328_cropped_by_bayernatlas_20260812084517_satellite_overlay.html
```

Or open the final full-cloud LZ overlay:

```bash
google-chrome file:///home/gvb/Documents/robotspace/DDLZD_ws/Drone-Delivery-Landing-Zone-Detection/pcds/696_5328_cropped_full_satellite_lz_overlay.html
```

Open the max-2000 full-cloud LZ overlay:

```bash
google-chrome file:///home/gvb/Documents/robotspace/DDLZD_ws/Drone-Delivery-Landing-Zone-Detection/pcds/696_5328_cropped_full_satellite_lz_overlay_max2000.html
```

Open the max-10000 exact-radius-2.5 LZ overlay:

```bash
google-chrome file:///home/gvb/Documents/robotspace/DDLZD_ws/Drone-Delivery-Landing-Zone-Detection/pcds/696_5328_cropped_full_satellite_lz_overlay_max10000_exact_radius2_5.html
```

Open the max-10000 radius-2.5-to-3.0 LZ overlay:

```bash
google-chrome file:///home/gvb/Documents/robotspace/DDLZD_ws/Drone-Delivery-Landing-Zone-Detection/pcds/696_5328_cropped_full_satellite_lz_overlay_max10000_radius2_5_to3.html
```

## Risk-ranked landing zones

After generating the radius-range candidates, run the orthophoto plus point-cloud risk-ranking script:

```bash
cd /home/gvb/Documents/robotspace/DDLZD_ws/Drone-Delivery-Landing-Zone-Detection

/home/gvb/Documents/robotspace/venvs/praktikum_flightplanning_venv/bin/python3 \
scripts/rank_landing_zone_risk.py \
--candidates pcds/region_growing_landing_zones_max10000_radius2_5_to3.csv \
--pcd pcds/696_5328_cropped_by_bayernatlas_20260812084517.pcd \
--classified-pcd pcds/696_5328_cropped_by_bayernatlas_20260812084517_classified.pcd \
--orthophoto pcds/696_5328_cropped_by_bayernatlas_20260812084517_satellite_overlay_dop20.png \
--output-prefix pcds/region_growing_landing_zones_max10000_radius2_5_to3_risk_ranked
```

The current run produced:

```text
Safest: 942
Safe:    244
Risky:   170
Total:  1356
```

Outputs:

```text
pcds/region_growing_landing_zones_max10000_radius2_5_to3_risk_ranked.csv
pcds/region_growing_landing_zones_max10000_radius2_5_to3_risk_ranked.html
pcds/region_growing_landing_zones_max10000_radius2_5_to3_risk_ranked_orthophoto_risk.png
pcds/region_growing_landing_zones_max10000_radius2_5_to3_risk_ranked_obstacle_offset.png
pcds/region_growing_landing_zones_max10000_radius2_5_to3_risk_ranked.tex
```

Open the ranked overlay:

```bash
google-chrome file:///home/gvb/Documents/robotspace/DDLZD_ws/Drone-Delivery-Landing-Zone-Detection/pcds/region_growing_landing_zones_max10000_radius2_5_to3_risk_ranked.html
```

The risk score combines:

```text
point-cloud obstacle count
offset to high points / orthophoto obstacle mask
orthophoto tree-like vegetation score
point-cloud relief and roughness
orthophoto texture
slope
LAS class-20 fraction where available
grass-like orthophoto bonus
GPS plus wind drift margin
```

The default UAV margin is:

```text
gps_error_m + wind_factor * wind_speed_mps / descent_speed_mps = 1.0 + 0.25 * 4.0 / 1.0 = 2.0 m
```

Adjust it with:

```bash
--gps-error-m 1.5 --wind-speed-mps 6.0 --descent-speed-mps 0.8 --wind-factor 0.25
```

Deep-learning variant:

```bash
--sam-checkpoint /path/to/sam_vit_h_4b8939.pth --sam-model-type vit_h
```

This only runs when the real Meta Segment Anything package and checkpoint are installed. If no checkpoint is supplied, the script reports a fully reproducible traditional computer-vision plus point-cloud fusion result.

## Ground-class candidate detection with full-cloud obstacle checking

If you want to generate landing-zone candidates from the class-2 ground-only PCD and then validate them against the full cropped point cloud, add this activation tag to `lib/config/algo_testing_own_pcd_config.yaml`:

```yaml
# TAG: USE_GROUND_CLASS2_FOR_DETECTION_AND_FULL_CLOUD_FOR_OBSTACLE_CHECK
```

Activation tags are selected through the `active_tags` list in `lib/config/algo_testing_own_pcd_config.yaml`. Commented `# TAG:` lines are documentation only.

If you want to run the radius-range experiment on all classes directly, use this tag and adjust `min_radius_threshold` / `max_radius_threshold`:

```yaml
active_tags:
  - USE_ALL_CLASSES_RADIUS_RANGE_EXPERIMENT
min_radius_threshold: 2.5
max_radius_threshold: 3.0
```

If you want to run the exact-radius 2.5 m experiment on all classes directly, use this tag. Candidate centers with at least 2.5 m clearance are kept, then the evaluated/exported radius is forced to exactly 2.5 m:

```yaml
active_tags:
  - USE_ALL_CLASSES_EXACT_RADIUS_2_5_ONLY_EXPERIMENT
```

Legacy exact-radius aliases:

```yaml
active_tags:
  - USE_RADIUS_2_5_ONLY_EXPERIMENT
  - USE_EXACT_RADIUS_2_5_ONLY_EXPERIMENT
```

Then run the helper script:

```bash
/home/gvb/Documents/robotspace/venvs/praktikum_flightplanning_venv/bin/python3 \
  scripts/check_lz_full_cloud_obstacle_check.py \
  --config lib/config/algo_testing_own_pcd_config.yaml \
  --candidate-csv pcds/region_growing_landing_zones_max2000.csv \
  --full-cloud-pcd pcds/696_5328_cropped_by_bayernatlas_20260812084517.pcd
```

That writes:

```text
pcds/region_growing_landing_zones_max2000_fullcloud_checked.csv
```

Generate the HTML viewer for the filtered result:

```bash
/home/gvb/Documents/robotspace/venvs/praktikum_flightplanning_venv/bin/python3 \
  scripts/visualize_pcd_gpx_mesh.py \
  --pcd pcds/696_5328_cropped_by_bayernatlas_20260812084517.pcd \
  --html pcds/696_5328_cropped_full_satellite_lz_overlay_max2000_fullcloud.html \
  --orthophoto pcds/696_5328_cropped_by_bayernatlas_20260812084517_satellite_overlay_dop20.png \
  --landing-zones-csv pcds/region_growing_landing_zones_max2000_fullcloud_checked.csv \
  --html-max-points 180000 \
  --orthophoto-stride 4 \
  --no-gpx
```

Open the full-cloud checked overlay:

```bash
google-chrome file:///home/gvb/Documents/robotspace/DDLZD_ws/Drone-Delivery-Landing-Zone-Detection/pcds/696_5328_cropped_full_satellite_lz_overlay_max2000_fullcloud.html
```

google-chrome file:///home/gvb/Documents/robotspace/DDLZD_ws/Drone-Delivery-Landing-Zone-Detection/pcds/696_5328_cropped_full_satellite_lz_overlay_max10000.html
