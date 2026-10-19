#!/usr/bin/env bash

set -euo pipefail

image="${DDLZD_IMAGE:-ddlzd-lidar-only:humble}"
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/.." && pwd)"
config_file="${DDLZD_CONFIG:-${repo_root}/ros2_ws/src/ddlzd_ros/config/live_fusion.yaml}"
tty_args=()
if [ -t 0 ]; then
  tty_args=(-it)
fi

xhost +local:root +local:docker >/dev/null 2>&1 || true

docker run --name ddlzd-lidar-only --rm "${tty_args[@]}" --privileged \
  --runtime nvidia --network host --ipc host \
  -e DISPLAY="${DISPLAY:-}" \
  -e XAUTHORITY="${XAUTHORITY:-}" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "${config_file}:/config/live_fusion.yaml:ro" \
  "${image}" "$@"
