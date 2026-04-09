#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(dirname "${BASH_SOURCE[0]}")"
cd "$SCRIPT_DIR"

if [[ ! -f devel/setup.bash ]]; then
  echo "[ERROR] Missing devel/setup.bash. Build workspace first (catkin_make)." >&2
  exit 1
fi

if ! command -v roslaunch >/dev/null 2>&1; then
  echo "[ERROR] roslaunch not found in PATH." >&2
  exit 1
fi

# Optional runtime overrides (real robot)
LIVOX_BD_LIST="${LIVOX_BD_LIST:-}"
LIVOX_FRAME="${LIVOX_FRAME:-livox_frame}"
FASTLIO_RVIZ="${FASTLIO_RVIZ:-false}"

# Compatibility bridge controls
ENABLE_TASK5_COMPAT_BRIDGE="${ENABLE_TASK5_COMPAT_BRIDGE:-true}"
ENABLE_CLOUD_TO_JH_CLOUD="${ENABLE_CLOUD_TO_JH_CLOUD:-true}"
ENABLE_ODOMETRY_TO_ODOM="${ENABLE_ODOMETRY_TO_ODOM:-true}"
ENABLE_CLOUD_TO_SCAN="${ENABLE_CLOUD_TO_SCAN:-true}"

# TF strategy defaults: livox_frame -> base_link_fusion enabled, base_link -> base_link_fusion disabled
ENABLE_TF_MAP_TO_ODOM_FUSION="${ENABLE_TF_MAP_TO_ODOM_FUSION:-true}"
ENABLE_TF_BODY_TO_BASE_LINK="${ENABLE_TF_BODY_TO_BASE_LINK:-true}"
ENABLE_TF_BASE_LINK_TO_BASE_LINK_FUSION="${ENABLE_TF_BASE_LINK_TO_BASE_LINK_FUSION:-false}"
ENABLE_TF_LIVOX_TO_BASE_LINK_FUSION="${ENABLE_TF_LIVOX_TO_BASE_LINK_FUSION:-true}"

# Optional livox extrinsic overrides (meters/radians)
LIVOX_TO_BASE_LINK_FUSION_X="${LIVOX_TO_BASE_LINK_FUSION_X:-0.35}"
LIVOX_TO_BASE_LINK_FUSION_Y="${LIVOX_TO_BASE_LINK_FUSION_Y:-0.0}"
LIVOX_TO_BASE_LINK_FUSION_Z="${LIVOX_TO_BASE_LINK_FUSION_Z:--0.8}"
LIVOX_TO_BASE_LINK_FUSION_YAW="${LIVOX_TO_BASE_LINK_FUSION_YAW:-0.0}"
LIVOX_TO_BASE_LINK_FUSION_PITCH="${LIVOX_TO_BASE_LINK_FUSION_PITCH:--0.5236}"
LIVOX_TO_BASE_LINK_FUSION_ROLL="${LIVOX_TO_BASE_LINK_FUSION_ROLL:-0.0}"

# shellcheck disable=SC1091
source devel/setup.bash

CMD=(
  roslaunch
  fast_lio
  task5_fastlio.launch
  "livox_frame:=${LIVOX_FRAME}"
  "fastlio_rviz:=${FASTLIO_RVIZ}"
  "enable_task5_compat_bridge:=${ENABLE_TASK5_COMPAT_BRIDGE}"
  "enable_cloud_to_jh_cloud:=${ENABLE_CLOUD_TO_JH_CLOUD}"
  "enable_odometry_to_odom:=${ENABLE_ODOMETRY_TO_ODOM}"
  "enable_cloud_to_scan:=${ENABLE_CLOUD_TO_SCAN}"
  "enable_tf_map_to_odom_fusion:=${ENABLE_TF_MAP_TO_ODOM_FUSION}"
  "enable_tf_body_to_base_link:=${ENABLE_TF_BODY_TO_BASE_LINK}"
  "enable_tf_base_link_to_base_link_fusion:=${ENABLE_TF_BASE_LINK_TO_BASE_LINK_FUSION}"
  "enable_tf_livox_to_base_link_fusion:=${ENABLE_TF_LIVOX_TO_BASE_LINK_FUSION}"
  "livox_to_base_link_fusion_x:=${LIVOX_TO_BASE_LINK_FUSION_X}"
  "livox_to_base_link_fusion_y:=${LIVOX_TO_BASE_LINK_FUSION_Y}"
  "livox_to_base_link_fusion_z:=${LIVOX_TO_BASE_LINK_FUSION_Z}"
  "livox_to_base_link_fusion_yaw:=${LIVOX_TO_BASE_LINK_FUSION_YAW}"
  "livox_to_base_link_fusion_pitch:=${LIVOX_TO_BASE_LINK_FUSION_PITCH}"
  "livox_to_base_link_fusion_roll:=${LIVOX_TO_BASE_LINK_FUSION_ROLL}"
)

if [[ -n "$LIVOX_BD_LIST" ]]; then
  CMD+=("livox_bd_list:=${LIVOX_BD_LIST}")
fi

# Allow additional roslaunch args from CLI
CMD+=("$@")

echo "[INFO] Launch FAST-LIO (real robot): ${CMD[*]}"
exec "${CMD[@]}"
