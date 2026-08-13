#!/usr/bin/env bash

set -euo pipefail

image="${DDLZD_IMAGE:-ddlzd-lidar-only:humble}"
xhost +local:docker >/dev/null 2>&1

docker run --name ddlzd-lidar-only --rm -it --privileged \
  --gpus all --network host --ipc host \
  -e DISPLAY="${DISPLAY:-}" \
  -e XAUTHORITY="${XAUTHORITY:-}" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  "${image}" "$@"
