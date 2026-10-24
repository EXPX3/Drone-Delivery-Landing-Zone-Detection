# Semantic Obstacle Segmentation Experiment

## Method

The camera is nadir-looking and the person and boxes are often only 10-30 pixels
wide. A fixed COCO YOLO11s instance-segmentation baseline was therefore compared
with promptable YOLOE segmentation. The final pipeline uses YOLOE prompts for a
person, black/white cargo boxes, cardboard packages, and a car. Every accepted
image mask is projected onto the nearest corrected LiDAR scan using the recorded
camera intrinsics, distortion, timestamped UAV pose, and rigid camera extrinsic.

Geometric occupancy remains the authoritative landing-safety layer. Semantics
adds interpretable class labels but cannot suppress an obstacle when the image
model misses it.

## Observations

| Experiment | Result |
|---|---|
| YOLO11s-seg, COCO, 120 uniformly sampled frames | 32 labeled frames; no person detections; small objects were commonly assigned unrelated COCO classes |
| YOLOE-11s-seg, five scene prompts, 120 frames, 1280 px | 43 frames produced detections at exploratory confidence 0.015; strong person confidence reached 0.91 and cargo-box confidence reached 0.39 |
| Runtime sample | 120 independent GPU inferences in 4.07 s (29.5 image/s, excluding one-time model loading) |
| Integrated ROS replay | `/ddlzd/semantic_obstacles` published synchronized RGB LiDAR points; a checked person interval produced four associated LiDAR returns in one scan |

Visual inspection confirmed the person mask across separated flight intervals
and box masks on both dark and light targets. A substring error that mapped
`cargo` to `car` was diagnosed during this inspection and replaced with explicit
box/package/crate matching.

## Limits

This bag has no hand-labeled semantic ground truth, so the figures above are
detection/runtimes and visual verification, not precision, recall, or mIoU.
Sparse Velodyne returns can produce only a few class-colored 3D points on a small
object. A publishable quantitative evaluation requires polygon labels on held-out
frames and 3D cuboid/point labels on synchronized scans.

## Reproducibility

The Docker recipe pins the text-encoder source revision. Configuration and launch
commands are in `scripts/semantic_lz/README.md`; generated weights are deliberately
ignored because they total approximately 600 MB.
