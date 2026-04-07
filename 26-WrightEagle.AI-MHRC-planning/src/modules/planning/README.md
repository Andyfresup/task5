# Planning Module

## Current Implementation Status

✅ **Implemented**:
- LLM client (`llm_client.py`)
- Prompt engineering (`prompts.py`)
- Data model definitions (`schemas.py`)
- Planner wrapper (`planner.py`)
- Predefined action set

⏳ **To implement**:
- Dynamic replanning (`replan()`)
- Complex task decomposition
- Multi-step sequence planning

## Predefined Actions

- `navigate`: move to a specified location
- `search`: search for an object
- `pick`: pick up an object (grasping)
- `place`: place an object (placement)
- `speak`: voice output
- `wait`: wait

## Core Flow

User input → LLM interprets intent → decomposes into an action sequence → outputs RobotDecision

## Future Extensions

- Improve replanning based on execution feedback
- Support multi-step complex tasks
- Add task priority management