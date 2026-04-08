# LLM-based Task Planning System

> A single-robot variant implementation of the MHRC (Multi-Heterogeneous Robot Collaboration) framework for RoboCup@Home competition

[![Python](https://img.shields.io/badge/Python-3.11+-blue.svg)](https://www.python.org/)


---

## Overview

This project implements a **single-robot variant of the MHRC framework** specifically designed for the RoboCup@Home competition. The system uses Large Language Models (LLMs) for natural language understanding and task planning, following a modular architecture with four core components: Observation, Memory, Planning, and Execution.

### Module Description

| Module | Functionality | Implementation Status |
|--------|--------------|----------------------|
| **Observation** | Collects information from navigation, perception, and manipulation components. | ✅ Basic Implementation |
| **Memory** | Records task execution history and feedback from the environment. | ✅ Basic Implementation |
| **Planning** | LLM-based task decomposition into predefined action sequences. | ✅ Complete Implementation |
| **Execution** | Action execution, status monitoring, and feedback collection. | ✅ Mock + Task5 ROS adapter |

---

## Project Structure

```plaintext
LLM/
├── src/                                      # Source code
│   ├── main.py                               # Entry point
│   ├── config.py                             # Configuration
│   ├── config_local.example.py               # Configuration template
│   ├── robot_controller.py                   # Main controller
│   └── modules/                              # Four core modules
│       ├── observation/                      # Observation module
│       │   ├── observer_interface.py         # Observer interface
│       │   └── observers.py                  # Concrete observer implementations
│       ├── memory/                           # Memory module
│       │   └── memory_manager.py             # Memory manager
│       ├── planning/                         # Planning module
│       │   ├── llm_client.py                 # LLM client
│       │   ├── prompts.py                    # Prompt templates
│       │   ├── schemas.py                    # Action schemas
│       │   └── planner.py                    # Task planner
│       └── execution/                        # Execution module
│           ├── executor.py                   # Action executor
│           ├── mock_robot.py                 # Mock robot (for testing)
│           ├── robot_interface.py            # Robot interface definition
│           └── feedback.py                   # Feedback collector
├── demo/                                     # Demo scripts
├── tests/                                    # Test cases
├── requirements.txt                          # Dependencies
└── setup.sh                                  # Setup script
```

---

## Quick Start

### 1. Environment Setup

**Using the automated setup script (Recommended):**

```bash
# Clone repository
git clone https://github.com/USTC-WrightEagle-AI/LLM.git
cd LLM

# Run setup script
bash setup.sh
```

The script will:
- Create a conda environment named `cade` with Python 3.11
- Install all dependencies from `requirements.txt`
- Create configuration file from template

**Manual setup:**

```bash
# Create conda environment
conda create -n cade python=3.11 -y
conda activate cade

# Install dependencies
pip install -r requirements.txt

# Copy configuration template
cp src/config_local.example.py src/config_local.py
```

### 2. LLM Configuration

**Option A: Cloud API (Recommended for development)**

```bash
# Edit configuration file
nano src/config_local.py

# Add your API key
# Supported providers: DeepSeek, Alibaba DashScope, OpenAI-compatible APIs
```

**Option B: Local Ollama**

```bash
# Install Ollama
curl -fsSL https://ollama.com/install.sh | sh

# Pull model
ollama pull qwen2.5:3b

# Configure in config_local.py
MODE = RunMode.LOCAL
```

### 3. Run the System

```bash
# Enter source directory
cd src

# Interactive mode
python main.py

# Test mode (predefined scenarios)
python main.py --test

# Demo mode
python main.py --demo

# Debug mode
python main.py --mode debug
```

### 3.1 Enable Task5 ROS Execution Adapter

By default, MHRC runs with `MockRobot`. To route actions to Task5 ROS topics:

1. Set `ENABLE_MOCK = False` in `src/config_local.py`.
2. Ensure ROS1 runtime and Task5 topics are available.
3. Optionally tune adapter topics via environment variables:

```bash
export MHRC_TASK5_GOAL_TOPIC=/move_base_simple/goal
export MHRC_TASK5_SEARCH_TOPIC=/person_following/search_cmd_vel
export MHRC_TASK5_SPEAK_TOPIC=/person_following/mhrc_tts_text
export MHRC_TASK5_PICK_TOPIC=/person_following/pick_request
export MHRC_TASK5_PLACE_TOPIC=/person_following/place_request

# ACK channels + timeout failure reporting
export MHRC_TASK5_NAV_ACK_TOPIC=/person_following/navigate_ack
export MHRC_TASK5_PICK_ACK_TOPIC=/person_following/pick_ack
export MHRC_TASK5_PLACE_ACK_TOPIC=/person_following/place_ack
export MHRC_TASK5_ACK_TIMEOUT=6.0
export MHRC_TASK5_ACK_REQUIRED=true
```

Optional location map override:

```bash
export MHRC_TASK5_LOCATION_MAP_JSON='{"bar":[0.0,0.0,0.0],"table":[4.2,1.1,1.57]}'
```

Replanning controls (in `src/config_local.py`):

```python
ENABLE_REPLAN_ON_FAILURE = True
MAX_REPLAN_ATTEMPTS = 1
```

### 3.2 Task5 Integration Features (Implemented)

The current codebase already includes these Task5-facing capabilities:

- A ROS execution adapter (`Task5ROSAdapter`) that maps MHRC actions to Task5 topics.
- ROS master reachability check before node initialization to avoid long blocking when `roscore` is down.
- ACK-based action confirmation for `navigate/pick/place` with request ID matching.
- Timeout failure reporting (`ack_timeout`) for missing ACK responses.
- Feedback-driven replanning path in `RobotController` + `Planner.replan(...)`.

### 3.3 Task5 Deployment Checklist (End-to-End)

Use this sequence when integrating MHRC with the Task5 runtime under `robocup26/`.

1. Start ROS master:

```bash
source /opt/ros/noetic/setup.bash
roscore
```

2. Run baseline health check from workspace root:

```bash
cd robocup26
source .venv/bin/activate
bash run_task5_all.sh --check
```

3. Configure adapter topics and ACK policy:

```bash
export MHRC_TASK5_GOAL_TOPIC=/move_base_simple/goal
export MHRC_TASK5_SEARCH_TOPIC=/person_following/search_cmd_vel
export MHRC_TASK5_SPEAK_TOPIC=/person_following/mhrc_tts_text
export MHRC_TASK5_PICK_TOPIC=/person_following/pick_request
export MHRC_TASK5_PLACE_TOPIC=/person_following/place_request

export MHRC_TASK5_NAV_ACK_TOPIC=/person_following/navigate_ack
export MHRC_TASK5_PICK_ACK_TOPIC=/person_following/pick_ack
export MHRC_TASK5_PLACE_ACK_TOPIC=/person_following/place_ack
export MHRC_TASK5_ACK_TIMEOUT=6.0

# Set to false during early bring-up if ACK publishers are not ready yet.
export MHRC_TASK5_ACK_REQUIRED=false
```

4. Run MHRC controller:

```bash
cd robocup26/26-WrightEagle.AI-MHRC-planning/src
python3 main.py
```

5. If external devices are not connected, logs about waiting for `/cloud_registered` or missing `/dev/ttyUSB0` are expected and do not indicate adapter failure.

6. After hardware is connected, validate input links:

```bash
source /opt/ros/noetic/setup.bash
rostopic info /cloud_registered
rostopic hz /cloud_registered
ls -l /dev/ttyUSB0
```

For Task5 semantic parsing via MHRC, set `FOOD_SEMANTIC_BACKEND=mhrc` in `task5_person_tracker` launcher environment.

### 4. Example Commands

```
User: Hello
User: Go to the kitchen
User: Help me find an apple
User: Place the apple on the table
User: status      # Check robot status
User: stats       # View statistics
User: quit        # Exit
```

---

## Predefined Action Set

Following the MHRC framework, our system decomposes natural language instructions into a sequence of predefined actions:

| Action | Description | Parameters | Example |
|--------|-------------|-----------|---------|
| `navigate` | Navigate to target location | `target`: location name or coordinates | `{"type": "navigate", "target": "kitchen"}` |
| `search` | Search for object in environment | `object_name`: name of target object | `{"type": "search", "object_name": "apple"}` |
| `pick` | Grasp target object | `object_name`, `object_id` (optional) | `{"type": "pick", "object_name": "bottle"}` |
| `place` | Place held object at location | `location`: target placement location | `{"type": "place", "location": "table"}` |
| `speak` | Output speech content | `content`: text to speak | `{"type": "speak", "content": "Task completed"}` |
| `wait` | Wait / No operation | `reason` (optional): reason for waiting | `{"type": "wait", "reason": "awaiting user"}` |

These actions are validated using **Pydantic schemas** to ensure correct format and type safety.