#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${SCRIPT_DIR}"

USE_RVIZ="false"
USE_JOINT_STATE_PUBLISHER="true"
USE_GAZEBO_GUI="true"
USE_VEL_CONTROLLER="true"
FREEZE_GAZEBO_MODEL="true"
CMD_VEL_TOPIC="/cmd_vel"
WORLD_ARG=""
GAZEBO_MASTER_PORT="11346"
DO_BUILD="auto"
DEMO_CMD_VEL="false"
DEMO_LINEAR_X="0.20"
DEMO_ANGULAR_Z="0.00"

print_help() {
  cat <<'EOF'
Go2 Gazebo fused launcher

Usage:
  ./start_go2_base.sh [options] [extra_ros2_launch_args...]

Options:
  --build               Force run colcon build before launch
  --skip-build          Skip build
  --rviz                Launch with RViz
  --no-rviz             Launch without RViz (default)
  --no-jsp              Disable joint_state_publisher
  --no-controller       Do not launch the Go2 velocity controller GUI
  --headless            Launch Gazebo server only, without gzclient
  --no-gazebo-gui       Same as --headless
  --gazebo-gui          Keep Gazebo GUI client enabled (default)
  --freeze              Lock leg joints in a standing pose while base can move (default)
  --no-freeze           Keep real movable leg joints; requires a gait/controller to stand
  --cmd-vel-topic TOPIC Set cmd_vel topic (default: /cmd_vel)
  --world PATH          Gazebo world file path
  --gazebo-port PORT    Gazebo master port (default: 11346)
  --demo-cmd-vel        Publish a constant test cmd_vel in background
  --demo-linear-x V     Demo linear.x (default: 0.20)
  --demo-angular-z W    Demo angular.z (default: 0.00)
  -h, --help            Show this help

Examples:
  ./start_go2_base.sh
  ./start_go2_base.sh --rviz
  ./start_go2_base.sh --no-freeze
  ./start_go2_base.sh --build
  ./start_go2_base.sh --world /opt/ros/humble/share/gazebo_ros/worlds/empty.world
  ./start_go2_base.sh --demo-cmd-vel --demo-linear-x 0.3
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
    --no-rviz)
      USE_RVIZ="false"
      shift
      ;;
    --rviz)
      USE_RVIZ="true"
      shift
      ;;
    --no-jsp)
      USE_JOINT_STATE_PUBLISHER="false"
      shift
      ;;
    --no-controller|--no-vel-controller)
      USE_VEL_CONTROLLER="false"
      shift
      ;;
    --gazebo-gui)
      USE_GAZEBO_GUI="true"
      shift
      ;;
    --headless|--no-gazebo-gui)
      USE_GAZEBO_GUI="false"
      shift
      ;;
    --freeze|--freeze-joints|--static)
      FREEZE_GAZEBO_MODEL="true"
      shift
      ;;
    --no-freeze|--free-joints)
      FREEZE_GAZEBO_MODEL="false"
      shift
      ;;
    --cmd-vel-topic)
      CMD_VEL_TOPIC="${2:?missing topic value}"
      shift 2
      ;;
    --world)
      WORLD_ARG="${2:?missing world path}"
      shift 2
      ;;
    --gazebo-port)
      GAZEBO_MASTER_PORT="${2:?missing port value}"
      shift 2
      ;;
    --demo-cmd-vel)
      DEMO_CMD_VEL="true"
      shift
      ;;
    --demo-linear-x)
      DEMO_LINEAR_X="${2:?missing value}"
      shift 2
      ;;
    --demo-angular-z)
      DEMO_ANGULAR_Z="${2:?missing value}"
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

port_is_listening() {
  local port="$1"
  # If this TCP connect succeeds, the port is already in use by some service.
  timeout 0.2 bash -c ">/dev/tcp/127.0.0.1/${port}" >/dev/null 2>&1
}

select_free_gazebo_port() {
  local base_port="$1"
  local p
  for ((p=base_port; p<base_port+30; p++)); do
    if ! port_is_listening "${p}"; then
      echo "${p}"
      return 0
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

ROS_SETUP="$(detect_ros_setup || true)"
if [[ -z "${ROS_SETUP}" ]]; then
  echo "[ERROR] Cannot find ROS setup at /opt/ros/<distro>/setup.bash"
  exit 1
fi
safe_source "${ROS_SETUP}"

if [[ "${DO_BUILD}" == "true" ]] || [[ "${DO_BUILD}" == "auto" && ( ! -f "${WS_DIR}/install/setup.bash" || ! -x "${WS_DIR}/install/go2_vel_controller/lib/go2_vel_controller/go2_vel_controller.py" ) ]]; then
  echo "[INFO] Building workspace: ${WS_DIR}"
  (
    cd "${WS_DIR}"
    colcon build --symlink-install
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
export GAZEBO_MODEL_DATABASE_URI=""
export LIBGL_ALWAYS_SOFTWARE=0
export GAZEBO_RENDER_ENGINE=ogre

if command -v nvidia-smi >/dev/null 2>&1; then
  export __GLX_VENDOR_LIBRARY_NAME=nvidia
  export __NV_PRIME_RENDER_OFFLOAD=1
fi

if port_is_listening "${GAZEBO_MASTER_PORT}"; then
  suggested_port="$(select_free_gazebo_port "${GAZEBO_MASTER_PORT}" || true)"
  if [[ -z "${suggested_port}" ]]; then
    echo "[ERROR] Cannot find a free Gazebo port near ${GAZEBO_MASTER_PORT}."
    exit 1
  fi
  echo "[WARN] Gazebo port ${GAZEBO_MASTER_PORT} is in use, switching to ${suggested_port}."
  GAZEBO_MASTER_PORT="${suggested_port}"
fi
export GAZEBO_MASTER_URI="http://127.0.0.1:${GAZEBO_MASTER_PORT}"
ros2 daemon stop >/dev/null 2>&1 || true

demo_pid=""
cleanup() {
  if [[ -n "${demo_pid}" ]]; then
    kill "${demo_pid}" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT

if [[ "${DEMO_CMD_VEL}" == "true" ]]; then
  echo "[INFO] Starting demo cmd_vel publisher on ${CMD_VEL_TOPIC}"
  ros2 topic pub --rate 10 "${CMD_VEL_TOPIC}" geometry_msgs/msg/Twist \
    "{linear: {x: ${DEMO_LINEAR_X}, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: ${DEMO_ANGULAR_Z}}}" \
    >/tmp/go2_demo_cmd_vel.log 2>&1 &
  demo_pid=$!
fi

LAUNCH_ARGS=(
  "use_rviz:=${USE_RVIZ}"
  "use_joint_state_publisher:=${USE_JOINT_STATE_PUBLISHER}"
  "use_gazebo_gui:=${USE_GAZEBO_GUI}"
  "use_vel_controller:=${USE_VEL_CONTROLLER}"
  "freeze_gazebo_model:=${FREEZE_GAZEBO_MODEL}"
  "cmd_vel_topic:=${CMD_VEL_TOPIC}"
)
if [[ -n "${WORLD_ARG}" ]]; then
  LAUNCH_ARGS+=("world:=${WORLD_ARG}")
fi

echo "[INFO] Launching fused sim: Gazebo(server+gui=${USE_GAZEBO_GUI}, freeze=${FREEZE_GAZEBO_MODEL})+RViz+d435i+controller=${USE_VEL_CONTROLLER}"
echo "[INFO] RMW=${RMW_IMPLEMENTATION}, ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}, GAZEBO_MASTER_URI=${GAZEBO_MASTER_URI}"
echo "[INFO] GAZEBO_MODEL_DATABASE_URI disabled for startup stability."
echo "[INFO] Render hints: GAZEBO_RENDER_ENGINE=${GAZEBO_RENDER_ENGINE}, LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE}"
exec ros2 launch go2_bringup go2_sim_fused_headless.launch.py \
  "${LAUNCH_ARGS[@]}" \
  "${EXTRA_LAUNCH_ARGS[@]}"
