#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
default_bag_relative="bags/Bagfilles_Feb13_2026/recovered_bag"
bag_path="${1:-/media/gvb/ssd24ubuntu/${default_bag_relative}}"
rate="${2:-1.0}"
fps="${3:-12.0}"
image="${DDLZD_RVIZ_IMAGE:-cargopack:2026}"
optimized_poses="${DDLZD_OPTIMIZED_POSES:-${repo_root}/pcds/bag_only_strip_optimized_poses.npz}"

if [[ $# -eq 0 && ! -f "${bag_path}/metadata.yaml" ]]; then
  shopt -s nullglob
  mounted_candidates=(/media/"${USER}"/ssd24ubuntu*/"${default_bag_relative}")
  shopt -u nullglob
  for candidate in "${mounted_candidates[@]}"; do
    if [[ -f "${candidate}/metadata.yaml" ]]; then
      bag_path="${candidate}"
      break
    fi
  done
fi

if [[ ! -f "${bag_path}/metadata.yaml" ]] || ! compgen -G "${bag_path}/*.db3" >/dev/null; then
  cat >&2 <<EOF
ERROR: No readable ROS 2 bag was found at:
  ${bag_path}

Mount the SSD first, for example:
  udisksctl mount -b /dev/disk/by-label/ssd24ubuntu

Then pass the mounted bag path explicitly if needed.
EOF
  exit 1
fi

if [[ ! -f "${optimized_poses}" ]]; then
  cat >&2 <<EOF
ERROR: The validated corrected-pose archive is missing:
  ${optimized_poses}

Generate it with the bag-only mapping pipeline documented in:
  scripts/bag_lio_mapping/README.md
EOF
  exit 1
fi

printf 'Using ROS 2 bag: %s\n' "${bag_path}"
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
  -v "${optimized_poses}:/optimized_poses.npz:ro" \
  -w /workspace \
  --entrypoint bash "${image}" -lc "
    source /opt/ros/humble/setup.bash
    cp scripts/lz_rviz_viewer/lz_replay.rviz /tmp/ddlzd_lz_replay.rviz
    python3 scripts/lz_rviz_viewer/replay_node.py --bag /bag --rate '${rate}' --fps '${fps}' \
      --optimized-poses /optimized_poses.npz &
    node_pid=\$!
    ros2 run image_view image_view --ros-args --remap image:=/ddlzd/camera_overlay &
    image_pid=\$!
    trap 'kill \$node_pid \$image_pid 2>/dev/null || true' EXIT INT TERM
    rviz2 -d /tmp/ddlzd_lz_replay.rviz
  "
