#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
image="${DDLZD_RVIZ_IMAGE:-cargopack:2026}"
xauthority="${XAUTHORITY:-$HOME/.Xauthority}"
xauth_args=()
if [[ -f "${xauthority}" ]]; then
  xauth_args=(-e XAUTHORITY="${xauthority}" -v "${xauthority}:${xauthority}:ro")
fi

xhost +local:docker >/dev/null 2>&1 || true
docker run --rm --name ddlzd-mapping-viewer --gpus all --network host --ipc host \
  -e DISPLAY="${DISPLAY:-:0}" \
  -e QT_X11_NO_MITSHM=1 \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  "${xauth_args[@]}" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "${repo_root}:/workspace" \
  -w /workspace --entrypoint bash "${image}" -lc '
    source /opt/ros/humble/setup.bash
    python3 scripts/lz_rviz_viewer/mapping_audit_viewer.py &
    publisher_pid=$!
    trap '\''kill $publisher_pid 2>/dev/null || true'\'' EXIT INT TERM
    rviz2 -d scripts/lz_rviz_viewer/mapping_audit.rviz
  '
