#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
default_bag_relative="bags/Bagfilles_Feb13_2026/recovered_bag"
bag_path="${1:-${DDLZD_BAG:-/media/${USER}/ssd24ubuntu/${default_bag_relative}}}"
rate="${2:-1.0}"
fps="${3:-12.0}"
semantic_display_mode="${4:-${DDLZD_SEMANTIC_DISPLAY_MODE:-camera_lidar_correlated_only}}"
orthophoto_mode="${5:-${DDLZD_ORTHOPHOTO:-on}}"
sensor_tag="${6:-${DDLZD_SENSOR_TAG:-recorded_velodyne}}"
camera_mode="${7:-${DDLZD_CAMERA_MODE:-auto}}"
image="${DDLZD_RVIZ_IMAGE:-cargopack:2026}"
gpu_mode="${DDLZD_GPU:-auto}"
optimized_poses="${DDLZD_OPTIMIZED_POSES:-${repo_root}/pcds/bag_only_strip_optimized_poses.npz}"
replay_config="${DDLZD_REPLAY_CONFIG:-${repo_root}/scripts/config/lz_camera_projection.yaml}"
os1_profile="${DDLZD_SENSOR_PROFILE:-${repo_root}/scripts/config/sensor_profiles/os1.yaml}"
static_pcd="${DDLZD_STATIC_PCD:-${repo_root}/pcds/696_5328_cropped_by_bayernatlas_20260812084517.pcd}"
ground_pcd="${DDLZD_GROUND_PCD:-${repo_root}/pcds/696_5328_cropped_ground_only.pcd}"
zones_csv="${DDLZD_ZONES_CSV:-${repo_root}/pcds/region_growing_landing_zones_max10000_radius2_5_to3_risk_ranked.csv}"
orthophoto="${DDLZD_ORTHOPHOTO_FILE:-${repo_root}/pcds/696_5328_cropped_by_bayernatlas_20260812084517_satellite_overlay_dop20.png}"
semantic_weights="${DDLZD_SEMANTIC_WEIGHTS:-${repo_root}/pcds/semantic_experiment/models/yoloe-11s-seg.pt}"

case "${orthophoto_mode}" in
  on)
    orthophoto_arg="--show-orthophoto"
    orthophoto_mount_args=(-v "${orthophoto}:/orthophoto.png:ro")
    ;;
  off)
    orthophoto_arg=""
    orthophoto_mount_args=()
    ;;
  *) echo "Orthophoto mode must be 'on' or 'off', got: ${orthophoto_mode}" >&2; exit 2 ;;
esac
case "${sensor_tag}" in
  recorded_velodyne)
    sensor_profile_arg=""
    optimized_cli_arg="--optimized-poses /optimized_poses.npz"
    optimized_mount_args=(-v "${optimized_poses}:/optimized_poses.npz:ro")
    ;;
  os1)
    sensor_profile_arg="--sensor-profile /sensor_profile.yaml"
    optimized_cli_arg=""
    optimized_mount_args=()
    ;;
  *) echo "Sensor tag must be 'recorded_velodyne' or 'os1', got: ${sensor_tag}" >&2; exit 2 ;;
esac

if ! command -v docker >/dev/null 2>&1; then
  echo "ERROR: Docker is not installed or is not on PATH." >&2
  exit 1
fi
if [[ ! -r "${replay_config}" ]]; then
  echo "ERROR: Replay config is missing or unreadable: ${replay_config}" >&2
  exit 1
fi
config_mount_args=(-v "${replay_config}:/replay_config.yaml:ro")
profile_mount_args=()
if [[ "${sensor_tag}" == os1 ]]; then
  if [[ ! -r "${os1_profile}" ]]; then
    echo "ERROR: OS1 sensor profile is missing or unreadable: ${os1_profile}" >&2
    exit 1
  fi
  profile_mount_args=(-v "${os1_profile}:/sensor_profile.yaml:ro")
fi
case "${camera_mode}" in
  auto|required|off) ;;
  *) echo "Camera mode must be 'auto', 'required', or 'off', got: ${camera_mode}" >&2; exit 2 ;;
esac
case "${gpu_mode}" in
  auto)
    if command -v nvidia-smi >/dev/null 2>&1 \
        && nvidia-smi >/dev/null 2>&1 \
        && docker info --format '{{json .Runtimes}}' 2>/dev/null | grep -q 'nvidia'; then
      gpu_args=(--gpus all)
      gpu_env_args=(
        -e NVIDIA_DRIVER_CAPABILITIES=all
        -e __NV_PRIME_RENDER_OFFLOAD=1
        -e __GLX_VENDOR_LIBRARY_NAME=nvidia
      )
    else
      gpu_args=()
      gpu_env_args=()
    fi
    ;;
  on)
    gpu_args=(--gpus all)
    gpu_env_args=(
      -e NVIDIA_DRIVER_CAPABILITIES=all
      -e __NV_PRIME_RENDER_OFFLOAD=1
      -e __GLX_VENDOR_LIBRARY_NAME=nvidia
    )
    ;;
  off)
    gpu_args=()
    gpu_env_args=()
    ;;
  *) echo "DDLZD_GPU must be 'auto', 'on', or 'off', got: ${gpu_mode}" >&2; exit 2 ;;
esac

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

required_assets=(
  "${static_pcd}"
  "${ground_pcd}"
  "${zones_csv}"
)
if [[ "${orthophoto_mode}" == on ]]; then
  required_assets+=("${orthophoto}")
fi
for asset in "${required_assets[@]}"; do
  if [[ ! -r "${asset}" ]]; then
    printf 'ERROR: Required replay artifact is missing or unreadable:\n  %s\n' "${asset}" >&2
    printf 'See scripts/semantic_lz/PORTABLE_SETUP.md for the artifact manifest.\n' >&2
    exit 1
  fi
done

semantic_mount_args=()
semantic_cli_arg=""
if [[ "${camera_mode}" != off && -r "${semantic_weights}" ]]; then
  semantic_model_dir="$(cd "$(dirname "${semantic_weights}")" && pwd)"
  semantic_model_name="$(basename "${semantic_weights}")"
  semantic_mount_args=(-v "${semantic_model_dir}:/semantic_models:ro")
  semantic_cli_arg="--semantic-weights /semantic_models/${semantic_model_name}"
elif [[ "${camera_mode}" == required ]]; then
  echo "ERROR: Camera fusion requires semantic weights: ${semantic_weights}" >&2
  exit 1
fi

if ! docker image inspect "${image}" >/dev/null 2>&1; then
  cat >&2 <<EOF
ERROR: Docker image '${image}' is not installed.
Load or build the runtime image as documented in:
  scripts/semantic_lz/PORTABLE_SETUP.md
EOF
  exit 1
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

if [[ "${sensor_tag}" == recorded_velodyne && ! -f "${optimized_poses}" ]]; then
  cat >&2 <<EOF
ERROR: The validated corrected-pose archive is missing:
  ${optimized_poses}

Generate it with the bag-only mapping pipeline documented in:
  scripts/bag_lio_mapping/README.md
EOF
  exit 1
fi

printf 'Using ROS 2 bag: %s\n' "${bag_path}"
printf 'Semantic display tag: %s\n' "${semantic_display_mode}"
printf 'Bayern orthophoto: %s\n' "${orthophoto_mode}"
printf 'LiDAR sensor tag: %s\n' "${sensor_tag}"
printf 'Camera mode: %s\n' "${camera_mode}"
printf 'GPU acceleration: %s\n' "$([[ ${#gpu_args[@]} -gt 0 ]] && echo on || echo off)"
printf 'Replay config: %s\n' "${replay_config}"
printf 'Static/Ground PCD: %s | %s\n' "${static_pcd}" "${ground_pcd}"
xauthority="${XAUTHORITY:-$HOME/.Xauthority}"
xauth_args=()
if [[ -f "${xauthority}" ]]; then
  xauth_args=(-e XAUTHORITY="${xauthority}" -v "${xauthority}:${xauthority}:ro")
fi

xhost +local:docker >/dev/null 2>&1 || true
docker run --rm --name ddlzd-lz-rviz "${gpu_args[@]}" --network host --ipc host \
  -e DISPLAY="${DISPLAY:-:0}" \
  -e QT_X11_NO_MITSHM=1 \
  "${gpu_env_args[@]}" \
  "${xauth_args[@]}" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "${repo_root}:/workspace" \
  -v "${bag_path}:/bag:ro" \
  -v "${static_pcd}:/static_map.pcd:ro" \
  -v "${ground_pcd}:/ground_map.pcd:ro" \
  -v "${zones_csv}:/landing_zones.csv:ro" \
  "${config_mount_args[@]}" \
  "${profile_mount_args[@]}" \
  "${semantic_mount_args[@]}" \
  "${orthophoto_mount_args[@]}" \
  "${optimized_mount_args[@]}" \
  -w /workspace \
  --entrypoint bash "${image}" -lc "
    source /opt/ros/humble/setup.bash
    cp scripts/lz_rviz_viewer/lz_replay.rviz /tmp/ddlzd_lz_replay.rviz
    python3 scripts/lz_rviz_viewer/replay_node.py --bag /bag --config /replay_config.yaml \
      --pcd /static_map.pcd --ground-pcd /ground_map.pcd --zones /landing_zones.csv \
      --orthophoto /orthophoto.png ${semantic_cli_arg} \
      --rate '${rate}' --fps '${fps}' \
      ${optimized_cli_arg} \
      --semantic-display-mode '${semantic_display_mode}' --camera-mode '${camera_mode}' \
      ${orthophoto_arg} ${sensor_profile_arg} &
    node_pid=\$!
    image_pid=''
    if [[ '${camera_mode}' != off ]]; then
      ros2 run image_view image_view --ros-args --remap image:=/ddlzd/camera_overlay &
      image_pid=\$!
    fi
    trap 'kill \$node_pid \${image_pid:-} 2>/dev/null || true' EXIT INT TERM
    rviz2 -d /tmp/ddlzd_lz_replay.rviz
  "
