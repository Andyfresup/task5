#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(dirname "${BASH_SOURCE[0]}")"
cd "$SCRIPT_DIR"

DEFAULT_SPEECH_MODULE_FILE="../26-WrightEagle.AI-Speech/src/tts/synthesizer.py"
SPEECH_MODULE_FILE="${SPEECH_MODULE_FILE:-$DEFAULT_SPEECH_MODULE_FILE}"
USE_SPEECH_MODULE="${USE_SPEECH_MODULE:-true}"
if [[ ! -f "$SPEECH_MODULE_FILE" ]]; then
    USE_SPEECH_MODULE=false
fi

DEFAULT_SPEECH_ASR_FILE="../26-WrightEagle.AI-Speech/src/asr/vad-whisper.py"
SPEECH_ASR_FILE="${SPEECH_ASR_FILE:-$DEFAULT_SPEECH_ASR_FILE}"
USE_SPEECH_ASR_MODULE="${USE_SPEECH_ASR_MODULE:-true}"
if [[ ! -f "$SPEECH_ASR_FILE" ]]; then
    USE_SPEECH_ASR_MODULE=false
fi

if [[ -z "${DETECTION_ENABLE_VOICE:-}" ]]; then
    if [[ "${USE_SPEECH_ASR_MODULE}" == "true" ]]; then
        DETECTION_ENABLE_VOICE=false
    else
        DETECTION_ENABLE_VOICE=true
    fi
fi
FOOD_ORDER_JSON_FILE="${FOOD_ORDER_JSON_FILE:-person_following/food_orders.json}"
FOOD_SEMANTIC_ENABLED="${FOOD_SEMANTIC_ENABLED:-true}"
FOOD_SEMANTIC_BACKEND="${FOOD_SEMANTIC_BACKEND:-ollama}"
FOOD_SEMANTIC_COMMAND="${FOOD_SEMANTIC_COMMAND:-}"
FOOD_SEMANTIC_COMMAND_USE_SHELL="${FOOD_SEMANTIC_COMMAND_USE_SHELL:-false}"
FOOD_SEMANTIC_MODEL_PATH="${FOOD_SEMANTIC_MODEL_PATH:-}"
FOOD_SEMANTIC_TASK="${FOOD_SEMANTIC_TASK:-text-generation}"
FOOD_SEMANTIC_TIMEOUT="${FOOD_SEMANTIC_TIMEOUT:-8.0}"
FOOD_SEMANTIC_OLLAMA_URL="${FOOD_SEMANTIC_OLLAMA_URL:-}"
FOOD_SEMANTIC_OLLAMA_MODEL="${FOOD_SEMANTIC_OLLAMA_MODEL:-llama3.2:1b}"
RETURN_ANCHOR_JSON_FILE="${RETURN_ANCHOR_JSON_FILE:-person_following/return_anchor.json}"
RETURN_TO_ANCHOR_ON_ORDER_CONFIRM="${RETURN_TO_ANCHOR_ON_ORDER_CONFIRM:-true}"
RETURN_TABLE_TRIGGER_DISTANCE="${RETURN_TABLE_TRIGGER_DISTANCE:-1.0}"
RETURN_TABLE_SEARCH_RADIUS="${RETURN_TABLE_SEARCH_RADIUS:-2.2}"
RETURN_TABLE_STOP_OFFSET="${RETURN_TABLE_STOP_OFFSET:-0.60}"
YOLO_PERCEPTION_DIR="${YOLO_PERCEPTION_DIR:-../26-WrightEagle.AI-YOLO-Perception}"
TABLE_FOOD_CHECK_ENABLED=true
TABLE_FOOD_CHECK_DELAY="${TABLE_FOOD_CHECK_DELAY:-1.0}"
TABLE_FOOD_DETECT_COMMAND="python3 realsenseinfer.py"
TABLE_FOOD_DETECT_USE_SHELL=false
TABLE_FOOD_DETECT_TIMEOUT="${TABLE_FOOD_DETECT_TIMEOUT:-5.0}"
TABLE_FOOD_DETECTION_JSON_FILE="$YOLO_PERCEPTION_DIR/detections.json"
TABLE_FOOD_USE_FUZZY_MODEL="${TABLE_FOOD_USE_FUZZY_MODEL:-true}"
TABLE_FOOD_FUZZY_BACKEND="${TABLE_FOOD_FUZZY_BACKEND:-reuse_order_backend}"
TABLE_FOOD_LEXICAL_THRESHOLD="${TABLE_FOOD_LEXICAL_THRESHOLD:-0.76}"
SERVING_TARGET_CAPTURE_TOPIC="${SERVING_TARGET_CAPTURE_TOPIC:-/person_following/serving_target_capture}"
SERVING_TARGET_CAPTURE_CMD_TOPIC="${SERVING_TARGET_CAPTURE_CMD_TOPIC:-/person_following/serving_target_capture_cmd}"
SERVING_TARGET_SNAPSHOT_JSON_FILE="${SERVING_TARGET_SNAPSHOT_JSON_FILE:-person_following/serving_target_snapshot.json}"
SERVING_TARGET_FACE_IMAGE_FILE="${SERVING_TARGET_FACE_IMAGE_FILE:-person_following/serving_target_face.jpg}"
SERVING_TARGET_FACE_META_FILE="${SERVING_TARGET_FACE_META_FILE:-person_following/serving_target_face_meta.json}"
CUSTOMER_DATA_ROOT="${CUSTOMER_DATA_ROOT:-person_following/service_customers}"
ACTIVE_CUSTOMER_FOLDER_TOPIC="${ACTIVE_CUSTOMER_FOLDER_TOPIC:-/person_following/active_customer_folder}"
SERVING_CUSTOMER_STATE_TOPIC="${SERVING_CUSTOMER_STATE_TOPIC:-/person_following/serving_customer_state}"

pids=()
cleanup() {
    for pid in "${pids[@]:-}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null || true
        fi
    done
}
trap cleanup EXIT INT TERM

python3 person_following/pointcloud_to_occupancy_grid.py \
    _cloud_topic:=/cloud_registered \
    _grid_topic:=/person_following/occupancy_grid \
    _target_frame:=map \
    _base_frame:=base_link \
    _grid_width:=20.0 \
    _grid_height:=20.0 \
    _resolution:=0.10 \
    _robot_clear_radius:=0.45 \
    _inflate_radius:=0.20 \
    _publish_rate:=15.0 &
 pids+=("$!")

python3 person_following/person_goal_publisher.py \
    _global_frame:=map \
    _base_frame:=base_link \
    _map_topic:=/person_following/occupancy_grid \
    _follow_distance:=1.1 \
    _goal_update_distance:=0.22 \
    _min_publish_interval:=0.3 \
    _min_candidate_radius:=0.9 \
    _max_candidate_radius:=1.6 \
    _radius_samples:=5 \
    _front_angle_span_deg:=90.0 \
    _angle_samples:=13 \
    _weight_distance:=2.4 \
    _weight_angle:=1.2 \
    _weight_robot:=0.8 \
    _robot_radius:=0.34 \
    _path_check_step:=0.05 \
    _occupancy_threshold:=50 \
    _switch_score_margin:=0.35 \
    _switch_score_ratio:=0.15 \
    _goal_reach_hold_time:=1.0 \
    _person_reacquire_distance:=0.45 \
    _person_reacquire_forward:=0.28 \
    _person_reacquire_lateral:=0.20 \
    _person_reacquire_heading_deg:=14.0 \
    _gaze_tracking_on_pause:=true \
    _gaze_track_linear:=true \
    _gaze_track_lateral:=true \
    _gaze_yaw_deadband_deg:=4.0 \
    _gaze_max_angular:=0.45 \
    _gaze_max_forward:=0.04 \
    _gaze_max_reverse:=0.05 \
    _gaze_max_lateral:=0.03 \
    _gaze_target_distance:=1.1 \
    _gaze_distance_tolerance:=0.12 \
    _gaze_collision_probe_distance:=0.55 \
    _gaze_person_timeout:=0.6 \
    _pause_prompt_enabled:=true \
    _pause_prompt_text:="What do you want" \
    _pause_prompt_use_speech_module:=${USE_SPEECH_MODULE} \
    _pause_prompt_speech_module_file:=${SPEECH_MODULE_FILE} \
    _pause_reply_listen_enabled:=true \
    _pause_reply_use_speech_module:=${USE_SPEECH_ASR_MODULE} \
    _pause_reply_speech_module_file:=${SPEECH_ASR_FILE} \
    _pause_reply_timeout:=6.0 \
    _pause_reply_start_delay:=1.2 \
    _pause_reply_reask_on_unrecognized:=true \
    _pause_reply_reask_text:="Can I beg you a pardon?" \
    _pause_reply_reask_max_attempts:=1 \
    _pause_reply_reask_listen_delay:=1.1 \
    _pause_reply_topic:=/person_following/pause_reply_text \
    _food_order_enabled:=true \
    _food_order_json_file:=${FOOD_ORDER_JSON_FILE} \
    _food_order_confirm_enabled:=true \
    "_food_order_confirm_template:=OK, I'll get {foods} for you" \
    _return_to_anchor_on_order_confirm:=${RETURN_TO_ANCHOR_ON_ORDER_CONFIRM} \
    _return_anchor_json_file:=${RETURN_ANCHOR_JSON_FILE} \
    _return_anchor_bridge_republish:=2 \
    _return_table_approach_enabled:=true \
    _return_table_trigger_distance:=${RETURN_TABLE_TRIGGER_DISTANCE} \
    _return_table_search_radius:=${RETURN_TABLE_SEARCH_RADIUS} \
    _return_table_stop_offset:=${RETURN_TABLE_STOP_OFFSET} \
    _table_food_check_enabled:=${TABLE_FOOD_CHECK_ENABLED} \
    _table_food_check_delay:=${TABLE_FOOD_CHECK_DELAY} \
    "_table_food_detect_command:=${TABLE_FOOD_DETECT_COMMAND}" \
    _table_food_detect_use_shell:=${TABLE_FOOD_DETECT_USE_SHELL} \
    _table_food_detect_timeout:=${TABLE_FOOD_DETECT_TIMEOUT} \
    _table_food_detect_workdir:=${YOLO_PERCEPTION_DIR} \
    _table_food_detection_json_file:=${TABLE_FOOD_DETECTION_JSON_FILE} \
    _table_food_use_fuzzy_model:=${TABLE_FOOD_USE_FUZZY_MODEL} \
    _table_food_fuzzy_backend:=${TABLE_FOOD_FUZZY_BACKEND} \
    _table_food_lexical_threshold:=${TABLE_FOOD_LEXICAL_THRESHOLD} \
    _serving_target_enabled:=true \
    _serving_target_snapshot_json_file:=${SERVING_TARGET_SNAPSHOT_JSON_FILE} \
    _serving_target_capture_topic:=${SERVING_TARGET_CAPTURE_TOPIC} \
    _serving_target_capture_cmd_topic:=${SERVING_TARGET_CAPTURE_CMD_TOPIC} \
    _customer_data_root:=${CUSTOMER_DATA_ROOT} \
    _active_customer_folder_topic:=${ACTIVE_CUSTOMER_FOLDER_TOPIC} \
    _serving_customer_state_topic:=${SERVING_CUSTOMER_STATE_TOPIC} \
    _gaze_stable_face_capture_enabled:=true \
    _food_semantic_enabled:=${FOOD_SEMANTIC_ENABLED} \
    _food_semantic_backend:=${FOOD_SEMANTIC_BACKEND} \
    "_food_semantic_command:=${FOOD_SEMANTIC_COMMAND}" \
    _food_semantic_command_use_shell:=${FOOD_SEMANTIC_COMMAND_USE_SHELL} \
    "_food_semantic_model_path:=${FOOD_SEMANTIC_MODEL_PATH}" \
    _food_semantic_transformers_task:=${FOOD_SEMANTIC_TASK} \
    _food_semantic_timeout:=${FOOD_SEMANTIC_TIMEOUT} \
    "_food_semantic_ollama_url:=${FOOD_SEMANTIC_OLLAMA_URL}" \
    _food_semantic_ollama_model:=${FOOD_SEMANTIC_OLLAMA_MODEL} \
    _run_rate_hz:=20.0 &
 pids+=("$!")

python3 person_following/cmd_vel_arbiter.py \
     _search_topic:=/person_following/search_cmd_vel \
     _nav_topic:=/cmd_vel_nav \
     _output_topic:=/cmd_vel \
     _search_timeout:=0.5 \
     _nav_timeout:=1.0 &
 pids+=("$!")

python3 person_following/person_detection_with_voice.py \
    _person_topic:=/person/base_link_3d_position \
    _frame_id:=base_link \
    _return_anchor_enabled:=true \
    _return_anchor_frame:=map \
    _return_anchor_base_frame:=base_link \
    _return_anchor_topic:=/person_following/return_anchor \
    _return_anchor_json_file:=${RETURN_ANCHOR_JSON_FILE} \
    _serving_target_capture_topic:=${SERVING_TARGET_CAPTURE_TOPIC} \
    _serving_target_capture_cmd_topic:=${SERVING_TARGET_CAPTURE_CMD_TOPIC} \
    _serving_target_face_image_file:=${SERVING_TARGET_FACE_IMAGE_FILE} \
    _serving_target_face_meta_file:=${SERVING_TARGET_FACE_META_FILE} \
    _customer_data_root:=${CUSTOMER_DATA_ROOT} \
    _active_customer_folder_topic:=${ACTIVE_CUSTOMER_FOLDER_TOPIC} \
    _serving_customer_state_topic:=${SERVING_CUSTOMER_STATE_TOPIC} \
    _show_debug:=false \
    _enable_voice:=${DETECTION_ENABLE_VOICE} \
    _whisper_model:=small \
    _enable_search_rotation:=true \
    _costmap_topic:=/person_following/occupancy_grid &
 pids+=("$!")

wait
