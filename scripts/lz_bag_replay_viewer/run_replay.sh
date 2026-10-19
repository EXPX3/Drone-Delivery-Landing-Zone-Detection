#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
bag_path="${1:-/media/gvb/ssd24ubuntu/bags/Bagfilles_Feb13_2026/recovered_bag}"
port="${2:-8765}"
python_env="/home/gvb/Documents/robotspace/venvs/praktikum_flightplanning_venv/bin/python"
viewer_path="${repo_root}/pcds/region_growing_landing_zones_live_bag_replay.html"

"${python_env}" "${repo_root}/scripts/lz_bag_replay_viewer/prepare_viewer.py" --output "${viewer_path}"

echo "Open http://127.0.0.1:${port} after the server starts."
docker run --rm \
  --user "$(id -u):$(id -g)" \
  -e ROS_LOG_DIR=/tmp/ros_logs \
  --network host \
  -v "${repo_root}:/workspace" \
  -v "${bag_path}:/bag:ro" \
  -w /workspace \
  ws_extract_images_or_vid_frombag:latest \
  bash -lc "source /opt/ros/humble/setup.bash && \
    python3 scripts/lz_bag_replay_viewer/replay_server.py \
      --bag /bag --port ${port} \
      --viewer pcds/region_growing_landing_zones_live_bag_replay.html"
