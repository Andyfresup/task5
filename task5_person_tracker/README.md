# Team Navigation Repo

This repository contains the person tracking stack for the Robocup@Home team.

## Main Structure

- `person_following`: Main detection and tracking modules.
  - `person_detection.py`: Raised-hand person detection with lock-on behavior.
  - `person_detection_with_voice.py`: Raised-hand + voice-call detection with lock-on behavior.
  - `person_tracker.py`: Publish follow goals to `/move_base_simple/goal`.
   - `person_goal_publisher.py`: Publish follow goals for navigation stack / far_planner.
   - `pointcloud_to_occupancy_grid.py`: Convert point cloud to rolling `OccupancyGrid`.
   - `cmd_vel_arbiter.py`: Prioritize search rotation over planner velocity.

## Installation

1. Clone this repository:
   ```bash
   git clone https://github.com/USTC-WrightEagle-AI/person_tracker.git
   cd person_tracker
   ```

2. Install ROS dependencies:
   ```bash
   chmod +x install_dependencies.sh
   ./install_dependencies.sh
   ```

3. Build workspace:
   ```bash
   catkin_make
   ```

4. Source environment:
   ```bash
   source devel/setup.bash
   ```

5. Install Python dependencies:
   ```bash
   pip install -r requirements.txt
   pip install ultralytics
   ```

6. Optional (voice-call detection dependencies):
   ```bash
   pip install faster-whisper pyaudio resampy silero-vad
   ```

## Usage

### 1) Raised-Hand Detection (Lock-On)

Start detection by gesture:

```bash
python person_following/person_detection.py
```

Behavior:

- Detect people who raise hand.
- If multiple people raise hands at the same time, choose the rightmost one.
- Once one target is selected, lock on that target only.
- Keep updating locked target coordinates even if hand is lowered.
- Ignore other people after lock.

### 2) Raised-Hand + Voice-Call Detection (Lock-On)

Start detection by gesture or call words (`waiter`, `robot`, `assistant`):

```bash
python person_following/person_detection_with_voice.py
```

Behavior:

- Trigger lock by either raised hand or voice call.
- If raised-hand candidates exist, still choose rightmost raised-hand person.
- If triggered by voice call and no raised-hand candidate, choose nearest visible person.
- After lock, do not switch to other gesture/call targets.

Example with parameters:

```bash
python person_following/person_detection_with_voice.py \
  _enable_voice:=true \
  _whisper_model:=small \
  _call_timeout:=1.0 \
  _show_debug:=true
```

For navigation-stack coexistence, disable search rotation so this node does not publish `/cmd_vel` while the planner is active:

```bash
python person_following/person_detection_with_voice.py \
   _enable_voice:=true \
   _enable_search_rotation:=false
```

By default, the search rotation is enabled and it publishes to `/person_following/search_cmd_vel`. It also subscribes to the local `OccupancyGrid` bridge topic `/person_following/occupancy_grid`. If nearby obstacles are detected, the node flips the spin direction instead of continuing in the same direction.

### 3) Follow via Navigation Goal

```bash
python person_following/person_detection.py
python person_following/person_tracker.py
```

### 4) Follow via Far Planner / Navigation Stack (Recommended)

```bash
python person_following/person_goal_publisher.py
```

Default behavior:

- Subscribe `/person/base_link_3d_position`.
- Transform person point to global frame (`map` by default).
- Publish follow goal to `/move_base_simple/goal`.
- Navigation stack / far_planner controls `/cmd_vel`.
- When robot reaches stable near-goal region, goal publishing pauses.
- During pause, gaze-tracking stage publishes low-speed tracking command to keep facing person.
- If person moves beyond threshold, resume normal goal publishing.

Optional runtime parameters:

```bash
python person_following/person_goal_publisher.py \
   _follow_distance:=1.0 \
   _goal_update_distance:=0.25 \
   _goal_topic:=/move_base_simple/goal
```

Useful pause/gaze parameters:

- `_goal_reach_distance` (default: `0.28`)
- `_goal_reach_hold_time` (default: `1.0`)
- `_person_reacquire_distance` (default: `0.45`)
- `_person_reacquire_forward` (default: `0.28`)
- `_person_reacquire_lateral` (default: `0.20`)
- `_person_reacquire_heading_deg` (default: `14.0`)
- `_gaze_tracking_on_pause` (default: `true`)
- `_gaze_track_linear` (default: `true`)
- `_gaze_track_lateral` (default: `true`)
- `_gaze_max_forward` (default: `0.04`)
- `_gaze_max_lateral` (default: `0.03`)
- `_run_rate_hz` (default: `20.0`, higher means more frequent gaze control updates)
- `_gaze_collision_probe_distance` (default: `0.55`, forward/reverse collision probe distance)

### 5) One-Click Person Follow Stack

This repository also provides a direct shell launcher that runs:

- `person_detection_with_voice.py`
- `pointcloud_to_occupancy_grid.py`
- `person_goal_publisher.py`
- `cmd_vel_arbiter.py`

```bash
bash run_task5_person_follow_voice.sh
```

If you prefer to stay inside `person_following/`, use:

```bash
bash person_following/run_task5_person_follow_voice.sh
```

The arbitration chain is:

- `person_detection_with_voice.py` -> `/person_following/search_cmd_vel`
- far_planner local planner -> `/cmd_vel_nav`
- `cmd_vel_arbiter.py` -> `/cmd_vel`

If you launch FAR Planner separately, use its task5 launch with `cmd_vel_output_topic:=/cmd_vel_nav` so the arbiter can own the final `/cmd_vel` output.

Semantic backend can be selected via environment variables before launch:

```bash
export FOOD_SEMANTIC_BACKEND=mhrc
export FOOD_SEMANTIC_MHRC_BASE_URL=http://127.0.0.1:11434/v1
export FOOD_SEMANTIC_MHRC_MODEL=qwen2.5:3b
export FOOD_SEMANTIC_MHRC_API_KEY=ollama
export FOOD_SEMANTIC_MHRC_ASYNC_WORKERS=1
export FOOD_SEMANTIC_MHRC_TIMEOUT=5.0
export FOOD_SEMANTIC_MHRC_FUSE_FAIL_THRESHOLD=3
export FOOD_SEMANTIC_MHRC_FUSE_COOLDOWN=15.0
bash run_task5_person_follow_voice.sh
```

Available values for `FOOD_SEMANTIC_BACKEND` are: `ollama`, `command`, `transformers`, and `mhrc`.

### 5.1 Task5 + MHRC Deployment Runbook

Use this checklist when you run Task5 with MHRC semantic parsing.

1. Bring up ROS master and run pre-check:

```bash
source /opt/ros/noetic/setup.bash
roscore
```

Open another terminal:

```bash
source /opt/ros/noetic/setup.bash
cd robocup26
source .venv/bin/activate
bash run_task5_all.sh --check
```

2. Enable MHRC semantic backend and launch Task5 stack:

```bash
cd robocup26
source .venv/bin/activate

export FOOD_SEMANTIC_BACKEND=mhrc
export FOOD_SEMANTIC_MHRC_BASE_URL=http://127.0.0.1:11434/v1
export FOOD_SEMANTIC_MHRC_MODEL=qwen2.5:3b
export FOOD_SEMANTIC_MHRC_API_KEY=ollama

export FOOD_SEMANTIC_MHRC_ASYNC_WORKERS=1
export FOOD_SEMANTIC_MHRC_TIMEOUT=5.0
export FOOD_SEMANTIC_MHRC_FUSE_FAIL_THRESHOLD=3
export FOOD_SEMANTIC_MHRC_FUSE_COOLDOWN=15.0
export FOOD_SEMANTIC_MHRC_STATS_LOG_INTERVAL=30.0

bash run_task5_person_follow_voice.sh
```

3. No-hardware debug mode is allowed. If camera/serial devices are not connected, logs about waiting for `/cloud_registered` or missing `/dev/ttyUSB0` are expected. Use:

```bash
bash run_task5_all.sh --person-only
```

4. After hardware is connected, validate input links:

```bash
source /opt/ros/noetic/setup.bash
rostopic info /cloud_registered
rostopic hz /cloud_registered
ls -l /dev/ttyUSB0
```

5. If you also enable the MHRC execution adapter (`26-WrightEagle.AI-MHRC-planning`), configure ACK behavior during bring-up:

```bash
export MHRC_TASK5_NAV_REQUEST_TOPIC=/person_following/navigate_request
export MHRC_TASK5_NAV_DELEGATE_TO_TASK5=true
export MHRC_TASK5_ACK_TIMEOUT=6.0
export MHRC_TASK5_ACK_REQUIRED=true

# Task5-side state gating (recommended defaults)
export NAVIGATE_REQUEST_TOPIC=/person_following/navigate_request
export NAVIGATE_ACK_TOPIC=/person_following/navigate_ack
export MHRC_NAV_STATE_GATING_ENABLED=true
export MHRC_NAV_FORCE_ACCEPT=false
export MHRC_NAV_ALLOW_LOCKED=false
export MHRC_NAV_REQUEST_TTL=30.0
export MHRC_NAV_DEBUG_STATE_OVERRIDE_ENABLED=false
export MHRC_NAV_DEBUG_STATE_OVERRIDE_TOPIC=/person_following/debug_state_override
```

Debug-only overrides for matrix testing:

```bash
export MHRC_TASK5_ACK_REQUIRED=false
export MHRC_NAV_DEBUG_STATE_OVERRIDE_ENABLED=true
```

6. Run Task5 navigate gate matrix cases (state injection + boundary checks):

```bash
cd /home/andy/robocup26
python3 task5_person_tracker/tools/nav_gate_matrix_runner.py \
   --suite all \
   --enable-debug-state \
   --ack-wait-timeout 3.0
```

### 6) PointCloud -> OccupancyGrid Bridge (for FAST-LIO / FAR)

If your stack does not provide `/map` or `/move_base/global_costmap/costmap`,
run this bridge node to publish an `OccupancyGrid` from `/cloud_registered`:

```bash
python person_following/pointcloud_to_occupancy_grid.py
```

Then run goal publisher with the generated grid topic:

```bash
python person_following/person_goal_publisher.py \
   _map_topic:=/person_following/occupancy_grid
```

## Notes

- `person_detection_with_voice.py` reuses the voice-recognition pipeline idea from `26-WrightEagle.AI-Speech` (Whisper + VAD).
- If voice dependencies are not installed, run `person_detection.py` for gesture-only mode.
