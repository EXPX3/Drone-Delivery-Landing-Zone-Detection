#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
bag_path="${1:-/media/gvb/ssd24ubuntu/bags/Bagfilles_Feb13_2026/recovered_bag}"
rate="${2:-1.0}"
fps="${3:-12.0}"
image="${DDLZD_RVIZ_IMAGE:-cargopack:2026}"
xauthority="${XAUTHORITY:-$HOME/.Xauthority}"
xauth_args=()
if [[ -f "${xauthority}" ]]; then
  xauth_args=(-e XAUTHORITY="${xauthority}" -v "${xauthority}:${xauthority}:ro")
fi

xhost +local:docker >/dev/null 2>&1 || true
docker run --rm --name ddlzd-lz-rviz --gpus all --network host --ipc host \
  -e DISPLAY="${DISPLAY:-:0}" \
  -e QT_X11_NO_MITSHM=1 \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e __NV_PRIME_RENDER_OFFLOAD=1 \
  -e __GLX_VENDOR_LIBRARY_NAME=nvidia \
  "${xauth_args[@]}" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "${repo_root}:/workspace" \
  -v "${bag_path}:/bag:ro" \
  -w /workspace \
  --entrypoint bash "${image}" -lc "
    source /opt/ros/humble/setup.bash
    cp scripts/lz_rviz_viewer/lz_replay.rviz /tmp/ddlzd_lz_replay.rviz
    python3 scripts/lz_rviz_viewer/replay_node.py --bag /bag --rate '${rate}' --fps '${fps}' &
    node_pid=\$!
    ros2 run image_view image_view --ros-args --remap image:=/ddlzd/camera_overlay &
    image_pid=\$!
    trap 'kill \$node_pid \$image_pid 2>/dev/null || true' EXIT INT TERM
    rviz2 -d /tmp/ddlzd_lz_replay.rviz
  "
