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
| **Execution** | Action execution, status monitoring, and feedback collection. | ✅ Mock Implementation |

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