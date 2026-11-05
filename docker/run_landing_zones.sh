#!/usr/bin/env bash

set -euo pipefail

usage() {
  sed -n '2,54p' "$0" | sed -n 's/^# \{0,1\}//p'
}

# Run live or recorded landing-zone detection in a foreground container.
#
# Usage:
#   run_landing_zones.sh live --sensor SENSOR --mode MODE --target-frame FRAME [options]
#   run_landing_zones.sh bag --bag PATH --mode MODE --target-frame FRAME [options]
#
# Required for live:
#   --sensor vlp16|os1_rev7|os1_rev8
#   --velodyne-ip IP                 required for vlp16
#   --sensor-hostname HOST           required for either Ouster profile
#
# Required for bag:
#   --bag PATH                       ROS 2 bag directory or MCAP file
#   --sensor auto|vlp16|os1_rev7|os1_rev8
#                                    optional; default auto. Auto refuses an
#                                    Ouster bag if its revision is not provable.
#
# Required for both:
#   --mode 1|2|ranking|sticky        1/ranking or 2/sticky
#   --target-frame FRAME             gravity-aligned local ENU frame
#
# Common options:
#   --config FILE                    detector YAML
#   --local-pose-topic TOPIC         default /mavros/local_position/pose
#   --gnss-topic TOPIC               default /mavros/global_position/global
#   --point-cloud-topic TOPIC        bag override for an ambiguous bag
#   --input-motion-compensated       upstream cloud is already deskewed
#   --no-rviz
#
# Live mounting options:
#   --lidar-mode MODE                Ouster mode; default 1024x10
#   --timestamp-mode MODE            Ouster clock mode; default TIME_FROM_ROS_TIME
#   --lidar-frame FRAME              VLP16 frame; default velodyne
#   --ouster-mount-frame FRAME       Ouster frame; default os_sensor
#   --static-lidar-transform X Y Z R P YAW
#   --base-frame FRAME               parent for the explicit static transform
#
# Bag options:
#   --rate RATE                      playback rate; default 1.0
#
# Ctrl-C gracefully stops the ROS launch and container.

if [[ $# -lt 1 ]]; then
  usage
  exit 2
fi
if [[ "$1" == "-h" || "$1" == "--help" ]]; then
  usage
  exit 0
fi

source_kind="$1"
shift
if [[ "$source_kind" != "live" && "$source_kind" != "bag" ]]; then
  echo "source must be 'live' or 'bag'" >&2
  usage
  exit 2
fi

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/.." && pwd)"
image="${DDLZD_IMAGE:-ddlzd-lidar-only:humble}"
config_file="${DDLZD_CONFIG:-${repo_root}/ros2_ws/src/ddlzd_ros/config/live_fusion.yaml}"
sensor=""
mode=""
target_frame=""
bag_path=""
sensor_hostname=""
velodyne_ip=""
point_cloud_topic=""
local_pose_topic="/mavros/local_position/pose"
gnss_topic="/mavros/global_position/global"
lidar_mode="1024x10"
timestamp_mode="TIME_FROM_ROS_TIME"
lidar_frame="velodyne"
ouster_mount_frame="os_sensor"
base_frame="base_link"
rate="1.0"
use_rviz="true"
motion_compensated="false"
publish_static_tf="false"
static_tf=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --sensor) sensor="$2"; shift 2 ;;
    --mode) mode="$2"; shift 2 ;;
    --target-frame) target_frame="$2"; shift 2 ;;
    --bag) bag_path="$2"; shift 2 ;;
    --config) config_file="$2"; shift 2 ;;
    --sensor-hostname) sensor_hostname="$2"; shift 2 ;;
    --velodyne-ip) velodyne_ip="$2"; shift 2 ;;
    --point-cloud-topic) point_cloud_topic="$2"; shift 2 ;;
    --local-pose-topic) local_pose_topic="$2"; shift 2 ;;
    --gnss-topic) gnss_topic="$2"; shift 2 ;;
    --lidar-mode) lidar_mode="$2"; shift 2 ;;
    --timestamp-mode) timestamp_mode="$2"; shift 2 ;;
    --lidar-frame) lidar_frame="$2"; shift 2 ;;
    --ouster-mount-frame) ouster_mount_frame="$2"; shift 2 ;;
    --base-frame) base_frame="$2"; shift 2 ;;
    --rate) rate="$2"; shift 2 ;;
    --no-rviz) use_rviz="false"; shift ;;
    --input-motion-compensated) motion_compensated="true"; shift ;;
    --static-lidar-transform)
      if [[ $# -lt 7 ]]; then
        echo "--static-lidar-transform requires X Y Z R P YAW" >&2
        exit 2
      fi
      static_tf=("$2" "$3" "$4" "$5" "$6" "$7")
      publish_static_tf="true"
      shift 7
      ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown option: $1" >&2; usage; exit 2 ;;
  esac
done

case "$mode" in
  1|ranking) mode="ranking" ;;
  2|sticky) mode="sticky" ;;
  *) echo "--mode must be 1, 2, ranking, or sticky" >&2; exit 2 ;;
esac
if [[ -z "$target_frame" ]]; then
  echo "--target-frame is required" >&2
  exit 2
fi
if [[ ! -f "$config_file" ]]; then
  echo "configuration file not found: $config_file" >&2
  exit 2
fi
config_file="$(realpath "$config_file")"

launch_args=(
  "operating_mode:=${mode}"
  "target_frame:=${target_frame}"
  "config_file:=/config/live_fusion.yaml"
  "local_pose_topic:=${local_pose_topic}"
  "gnss_topic:=${gnss_topic}"
  "input_is_motion_compensated:=${motion_compensated}"
  "use_rviz:=${use_rviz}"
)
volume_args=(-v "${config_file}:/config/live_fusion.yaml:ro")

if [[ "$source_kind" == "live" ]]; then
  if [[ "$sensor" != "vlp16" && "$sensor" != "os1_rev7" && "$sensor" != "os1_rev8" ]]; then
    echo "live --sensor must be vlp16, os1_rev7, or os1_rev8" >&2
    exit 2
  fi
  if [[ "$sensor" == "vlp16" && -z "$velodyne_ip" ]]; then
    echo "--velodyne-ip is required for vlp16" >&2
    exit 2
  fi
  if [[ "$sensor" != "vlp16" && -z "$sensor_hostname" ]]; then
    echo "--sensor-hostname is required for Ouster" >&2
    exit 2
  fi
  launch_file="live_lidar.launch.py"
  launch_args+=(
    "sensor_model:=${sensor}"
    "lidar_mode:=${lidar_mode}"
    "timestamp_mode:=${timestamp_mode}"
    "lidar_frame:=${lidar_frame}"
    "ouster_mount_frame:=${ouster_mount_frame}"
    "base_frame:=${base_frame}"
    "publish_lidar_static_tf:=${publish_static_tf}"
  )
  if [[ "$sensor" == "vlp16" ]]; then
    launch_args+=("velodyne_ip:=${velodyne_ip}")
  else
    launch_args+=("sensor_hostname:=${sensor_hostname}")
  fi
  if [[ "$publish_static_tf" == "true" ]]; then
    launch_args+=(
      "lidar_x:=${static_tf[0]}" "lidar_y:=${static_tf[1]}" "lidar_z:=${static_tf[2]}"
      "lidar_roll:=${static_tf[3]}" "lidar_pitch:=${static_tf[4]}" "lidar_yaw:=${static_tf[5]}"
    )
  fi
else
  if [[ -z "$bag_path" || ! -e "$bag_path" ]]; then
    echo "bag --bag must name an existing bag directory or MCAP file" >&2
    exit 2
  fi
  if [[ -z "$sensor" ]]; then
    sensor="auto"
  fi
  if [[ "$sensor" != "auto" && "$sensor" != "vlp16" && \
        "$sensor" != "os1_rev7" && "$sensor" != "os1_rev8" ]]; then
    echo "bag --sensor must be auto, vlp16, os1_rev7, or os1_rev8" >&2
    exit 2
  fi
  bag_path="$(realpath "$bag_path")"
  if [[ -d "$bag_path" ]]; then
    container_bag="/bag/input"
  else
    container_bag="/bag/$(basename "$bag_path")"
  fi
  volume_args+=(-v "${bag_path}:${container_bag}:ro")
  launch_file="bag_landing_zones.launch.py"
  launch_args+=("bag_path:=${container_bag}" "sensor_model:=${sensor}" "playback_rate:=${rate}")
  if [[ -n "$point_cloud_topic" ]]; then
    launch_args+=("point_cloud_topic:=${point_cloud_topic}")
  fi
fi

tty_args=()
if [[ -t 0 ]]; then
  tty_args=(-it)
fi
if [[ "$use_rviz" == "true" ]]; then
  xhost +local:root +local:docker >/dev/null 2>&1 || true
fi

container_name="ddlzd-landing-zones-$$"
stop_container() {
  docker stop --time 10 "$container_name" >/dev/null 2>&1 || true
}
trap stop_container INT TERM

set +e
docker run --name "$container_name" --rm "${tty_args[@]}" --privileged \
  --runtime nvidia --network host --ipc host \
  -e DISPLAY="${DISPLAY:-}" -e XAUTHORITY="${XAUTHORITY:-}" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  "${volume_args[@]}" \
  "$image" ros2 launch ddlzd_ros "$launch_file" "${launch_args[@]}"
status=$?
set -e
trap - INT TERM
exit "$status"
