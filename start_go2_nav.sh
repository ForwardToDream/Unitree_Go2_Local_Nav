#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${SCRIPT_DIR}"

DO_BUILD="auto"
USE_INTENT_UI="true"
USE_RVIZ="true"
TARGET_FRAME="base_link"
LIDAR_CLOUD_TOPIC="/unilidar/cloud"
DEPTH_CLOUD_TOPIC="/go2/camera/depth/points"
CMD_VEL_TOPIC="/cmd_vel"
INTENT_TOPIC="/mdog/owner_intent"
FUSED_POINTS_TOPIC="/mdog/fused_points"
LOCAL_GRID_TOPIC="/mdog/local_grid"
TRAVERSABILITY_TOPIC="/mdog/traversability"
RESOLUTION="0.10"
ROI_FRONT_M="4.0"
ROI_BACK_M="0.8"
ROI_SIDE_M="2.0"
HUMAN_WIDTH_M="0.8"
HUMAN_HEIGHT_M="1.8"
SAFETY_MARGIN_M="0.15"
GROUND_Z_IN_BASE_M="-0.35"
STALE_TIMEOUT_SEC="0.5"

MDOG_PACKAGES=(
  mdog_interfaces
  mdog_owner_intent_ui
  mdog_owner_model
  mdog_pointcloud_fusion
  mdog_semantic_map
  mdog_traversability
  mdog_local_planner
  mdog_bringup
)

print_help() {
  cat <<'EOF'
MDog local semantic navigation launcher

Usage:
  ./start_go2_nav.sh [options] [extra_ros2_launch_args...]

Options:
  --build                    Force build MDog packages before launch
  --skip-build               Skip build
  --ui                       Launch owner intent UI (default)
  --no-ui                    Launch without owner intent UI
  --rviz                     Launch MDog RViz semantic view (default)
  --no-rviz                  Launch without MDog RViz
  --target-frame FRAME       Local navigation frame (default: base_link)
  --lidar-cloud-topic TOPIC  Lidar PointCloud2 topic (default: /unilidar/cloud)
  --depth-cloud-topic TOPIC  Depth PointCloud2 topic (default: /go2/camera/depth/points)
  --cmd-vel-topic TOPIC      Output cmd_vel topic (default: /cmd_vel)
  --intent-topic TOPIC       Owner intent topic (default: /mdog/owner_intent)
  --resolution M             Local grid resolution (default: 0.10)
  --roi-front M              Forward ROI distance (default: 4.0)
  --roi-back M               Rear ROI distance (default: 0.8)
  --roi-side M               Half-width ROI distance (default: 2.0)
  --human-width M            Owner passage width envelope (default: 0.8)
  --human-height M           Owner passage height envelope (default: 1.8)
  --safety-margin M          Traversability margin around owner envelope (default: 0.15)
  --ground-z M               Ground z in base frame (default: -0.35)
  --stale-timeout SEC        Planner sensor stale timeout (default: 0.5)
  -h, --help                 Show this help

Examples:
  ./start_go2_nav.sh
  ./start_go2_nav.sh --no-ui
  ./start_go2_nav.sh --no-rviz
  ./start_go2_nav.sh --build
  ./start_go2_nav.sh --human-width 0.9 --safety-margin 0.2
  ./start_go2_nav.sh --no-ui --show-args

Notes:
  Start the Go2 simulation base separately with ./start_go2_base.sh.
  This launcher starts the MDog navigation stack and connects to the existing Go2 topics.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --build)
      DO_BUILD="true"
      shift
      ;;
    --skip-build)
      DO_BUILD="false"
      shift
      ;;
    --ui)
      USE_INTENT_UI="true"
      shift
      ;;
    --no-ui)
      USE_INTENT_UI="false"
      shift
      ;;
    --rviz)
      USE_RVIZ="true"
      shift
      ;;
    --no-rviz)
      USE_RVIZ="false"
      shift
      ;;
    --target-frame)
      TARGET_FRAME="${2:?missing frame value}"
      shift 2
      ;;
    --lidar-cloud-topic)
      LIDAR_CLOUD_TOPIC="${2:?missing topic value}"
      shift 2
      ;;
    --depth-cloud-topic)
      DEPTH_CLOUD_TOPIC="${2:?missing topic value}"
      shift 2
      ;;
    --cmd-vel-topic)
      CMD_VEL_TOPIC="${2:?missing topic value}"
      shift 2
      ;;
    --intent-topic)
      INTENT_TOPIC="${2:?missing topic value}"
      shift 2
      ;;
    --resolution)
      RESOLUTION="${2:?missing resolution value}"
      shift 2
      ;;
    --roi-front)
      ROI_FRONT_M="${2:?missing distance value}"
      shift 2
      ;;
    --roi-back)
      ROI_BACK_M="${2:?missing distance value}"
      shift 2
      ;;
    --roi-side)
      ROI_SIDE_M="${2:?missing distance value}"
      shift 2
      ;;
    --human-width)
      HUMAN_WIDTH_M="${2:?missing width value}"
      shift 2
      ;;
    --human-height)
      HUMAN_HEIGHT_M="${2:?missing height value}"
      shift 2
      ;;
    --safety-margin)
      SAFETY_MARGIN_M="${2:?missing margin value}"
      shift 2
      ;;
    --ground-z)
      GROUND_Z_IN_BASE_M="${2:?missing z value}"
      shift 2
      ;;
    --stale-timeout)
      STALE_TIMEOUT_SEC="${2:?missing timeout value}"
      shift 2
      ;;
    -h|--help)
      print_help
      exit 0
      ;;
    *)
      break
      ;;
  esac
done

EXTRA_LAUNCH_ARGS=("$@")

detect_ros_setup() {
  if [[ -n "${ROS_DISTRO:-}" ]] && [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    echo "/opt/ros/${ROS_DISTRO}/setup.bash"
    return
  fi

  local distro
  for distro in humble jazzy foxy; do
    if [[ -f "/opt/ros/${distro}/setup.bash" ]]; then
      echo "/opt/ros/${distro}/setup.bash"
      return
    fi
  done

  return 1
}

safe_source() {
  local file="$1"
  local restore_nounset="false"

  if [[ $- == *u* ]]; then
    restore_nounset="true"
    set +u
  fi

  # shellcheck disable=SC1090
  source "${file}"

  if [[ "${restore_nounset}" == "true" ]]; then
    set -u
  fi
}

mdog_install_ready() {
  [[ -f "${WS_DIR}/install/setup.bash" ]] \
    && [[ -x "${WS_DIR}/install/mdog_owner_intent_ui/lib/mdog_owner_intent_ui/mdog_owner_intent_ui.py" ]] \
    && [[ -x "${WS_DIR}/install/mdog_owner_model/lib/mdog_owner_model/mdog_owner_model_node" ]] \
    && [[ -x "${WS_DIR}/install/mdog_pointcloud_fusion/lib/mdog_pointcloud_fusion/mdog_pointcloud_fusion_node" ]] \
    && [[ -x "${WS_DIR}/install/mdog_semantic_map/lib/mdog_semantic_map/mdog_semantic_map_node" ]] \
    && [[ -x "${WS_DIR}/install/mdog_traversability/lib/mdog_traversability/mdog_traversability_node" ]] \
    && [[ -x "${WS_DIR}/install/mdog_local_planner/lib/mdog_local_planner/mdog_local_planner_node" ]] \
    && [[ -f "${WS_DIR}/install/mdog_bringup/share/mdog_bringup/launch/mdog_local_nav.launch.py" ]]
}

ROS_SETUP="$(detect_ros_setup || true)"
if [[ -z "${ROS_SETUP}" ]]; then
  echo "[ERROR] Cannot find ROS setup at /opt/ros/<distro>/setup.bash"
  exit 1
fi
safe_source "${ROS_SETUP}"

if [[ "${DO_BUILD}" == "true" ]] || [[ "${DO_BUILD}" == "auto" && ! mdog_install_ready ]]; then
  echo "[INFO] Building MDog packages in workspace: ${WS_DIR}"
  (
    cd "${WS_DIR}"
    colcon build --symlink-install --packages-select "${MDOG_PACKAGES[@]}"
  )
fi

if [[ ! -f "${WS_DIR}/install/setup.bash" ]]; then
  echo "[ERROR] Missing ${WS_DIR}/install/setup.bash, build failed or not completed."
  exit 1
fi
safe_source "${WS_DIR}/install/setup.bash"

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo" priority="default" multicast="false" /></Interfaces></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'

ros2 daemon stop >/dev/null 2>&1 || true

LAUNCH_ARGS=(
  "target_frame:=${TARGET_FRAME}"
  "lidar_cloud_topic:=${LIDAR_CLOUD_TOPIC}"
  "depth_cloud_topic:=${DEPTH_CLOUD_TOPIC}"
  "cmd_vel_topic:=${CMD_VEL_TOPIC}"
  "use_intent_ui:=${USE_INTENT_UI}"
  "use_rviz:=${USE_RVIZ}"
  "intent_topic:=${INTENT_TOPIC}"
  "fused_points_topic:=${FUSED_POINTS_TOPIC}"
  "local_grid_topic:=${LOCAL_GRID_TOPIC}"
  "traversability_topic:=${TRAVERSABILITY_TOPIC}"
  "resolution:=${RESOLUTION}"
  "roi_front_m:=${ROI_FRONT_M}"
  "roi_back_m:=${ROI_BACK_M}"
  "roi_side_m:=${ROI_SIDE_M}"
  "human_width_m:=${HUMAN_WIDTH_M}"
  "human_height_m:=${HUMAN_HEIGHT_M}"
  "safety_margin_m:=${SAFETY_MARGIN_M}"
  "ground_z_in_base_m:=${GROUND_Z_IN_BASE_M}"
  "stale_timeout_sec:=${STALE_TIMEOUT_SEC}"
)

echo "[INFO] Launching MDog local semantic navigation stack"
echo "[INFO] Sensors: lidar=${LIDAR_CLOUD_TOPIC}, depth=${DEPTH_CLOUD_TOPIC}, target_frame=${TARGET_FRAME}"
echo "[INFO] Output: cmd_vel=${CMD_VEL_TOPIC}, owner_intent=${INTENT_TOPIC}, ui=${USE_INTENT_UI}, rviz=${USE_RVIZ}"
echo "[INFO] Envelope: human_width=${HUMAN_WIDTH_M}m, human_height=${HUMAN_HEIGHT_M}m, safety_margin=${SAFETY_MARGIN_M}m"
echo "[INFO] RMW=${RMW_IMPLEMENTATION}, ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"
exec ros2 launch mdog_bringup mdog_local_nav.launch.py \
  "${LAUNCH_ARGS[@]}" \
  "${EXTRA_LAUNCH_ARGS[@]}"
