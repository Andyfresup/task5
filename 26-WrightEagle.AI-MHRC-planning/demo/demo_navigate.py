#!/usr/bin/env python3
"""
Complete Navigate Command Flow Demonstration

Shows every step from user input to robot execution
"""

import sys
import os

# Add project root and src directories to path
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)
sys.path.insert(0, os.path.join(project_root, 'src'))

import json
from modules.planning.schemas import NavigateAction, parse_action
from modules.execution.mock_robot import MockRobot


def demo_flow():
    """Complete flow demonstration"""

    print("="*70)
    print("🎬 Complete Navigate Command Flow Demonstration")
    print("="*70)

    # ==================== Phase 1: User Input ====================
    print("\n【Phase 1: User Input (Natural Language)】")
    print("─"*70)
    user_input = "Go to kitchen"
    print(f"👤 User says: \"{user_input}\"")
    print(f"\n💭 This is just normal human language, the robot cannot understand it directly")

    input("\nPress Enter to continue...")

    # ==================== Phase 2: System Prompt ====================
    print("\n\n【Phase 2: System Prompt Provides \"Manual\"】")
    print("─"*70)

    prompt_snippet = """
You have the following action capabilities:

1. **navigate** - Navigate to specified location
   - Parameter: target (e.g. "kitchen" or [x,y,z])
   - Example: {"type": "navigate", "target": "kitchen"}

Output format:
{
  "thought": "thinking process",
  "reply": "response",
  "action": {"type": "action_type", "param": "value"}
}
    """
    print(f"📖 System Prompt (snippet):\n{prompt_snippet}")

    print(f"\n💡 This tells the LLM:")
    print(f"   ✅ What actions are available")
    print(f"   ✅ What parameters each action needs")
    print(f"   ✅ How to output JSON format")

    input("\nPress Enter to continue...")

    # ==================== Phase 3: LLM Reasoning ====================
    print("\n\n【Phase 3: LLM Reasoning】")
    print("─"*70)

    print(f"\n🧠 LLM inner monologue:")
    print(f"   1. 'User said \"Go to kitchen\"'")
    print(f"   2. 'This is a request to move to a location'")
    print(f"   3. 'Check action list... navigate fits!'")
    print(f"   4. 'navigate needs a target parameter'")
    print(f"   5. '\"kitchen\" is the target'")
    print(f"   6. 'Output JSON'")

    llm_output = {
        "thought": "The user requests I move to the kitchen; this is a clear navigation command",
        "reply": "Okay, I'll go to the kitchen",
        "action": {
            "type": "navigate",
            "target": "kitchen"
        }
    }

    print(f"\n📤 LLM output:")
    print(json.dumps(llm_output, indent=2, ensure_ascii=False))

    input("\nPress Enter to continue...")

    # ==================== Phase 4: Schema Validation ====================
    print("\n\n【Phase 4: Pydantic Schema Validation】")
    print("─"*70)

    print(f"\n📋 Schema definition (brain/schemas.py):")
    schema_code = '''
class NavigateAction(BaseModel):
    type: Literal["navigate"] = "navigate"
    target: Union[str, List[float]]

    @field_validator('target')
    def validate_target(cls, v):
        if isinstance(v, list):
            if len(v) != 3:
                raise ValueError("Coordinates must be [x, y, z]")
        return v
    '''
    print(schema_code)

    print(f"\n🔍 Validation process:")

    try:
        action_data = llm_output["action"]
        print(f"   Input: {action_data}")

        # Validate
        action = parse_action(action_data)
        print(f"   ✅ type check: '{action.type}' == 'navigate'")
        print(f"   ✅ target check: '{action.target}' is a string")
        print(f"   ✅ Validation passed!")

        print(f"\n📦 Created object:")
        print(f"   action.type = '{action.type}'")
        print(f"   action.target = '{action.target}'")

    except Exception as e:
        print(f"   ❌ Validation failed: {e}")

    input("\nPress Enter to continue...")

    # ==================== Phase 5: Action Dispatch ====================
    print("\n\n【Phase 5: Action Dispatch】")
    print("─"*70)

    dispatch_code = '''
def _execute_action(self, action: RobotAction) -> bool:
    action_type = action.type  # "navigate"

    if action_type == "navigate":
        return self.robot.navigate(action.target)
                                    ↑
                            passing in "kitchen"
    '''
    print(f"📝 Code logic (robot_controller.py):")
    print(dispatch_code)

    print(f"\n🔀 Dispatch flow:")
    print(f"   1. Read action.type = '{action.type}'")
    print(f"   2. Match if action_type == \"navigate\"")
    print(f"   3. Call robot.navigate('{action.target}')")

    input("\nPress Enter to continue...")

    # ==================== Phase 6: Robot Execution ====================
    print("\n\n【Phase 6: Robot Execution】")
    print("─"*70)

    print(f"\n🤖 Create robot instance:")
    robot = MockRobot(name="DEMO")

    print(f"\n📍 Current position: {robot.current_position}")
    print(f"🗺️  Known locations: {list(robot.known_locations.keys())[:6]}...")

    print(f"\n⚡ Executing robot.navigate('kitchen'):")
    success = robot.navigate("kitchen")

    if success:
        print(f"\n✅ Execution succeeded!")
        print(f"📍 New position: {robot.current_position}")
    else:
        print(f"\n❌ Execution failed")

    # ==================== Full Flow Diagram ====================
    print("\n\n" + "="*70)
    print("📊 Full Flow Summary")
    print("="*70)

    flow_diagram = '''
┌────────────────────────────────────────────────────┐
│  User: "Go to kitchen"                             │
│  (Natural language)                                │
└─────────────┬──────────────────────────────────────┘
              ↓
┌─────────────────────────────────────────────────────┐
│  System Prompt                                      │
│  Tells the LLM what navigate is and how to use it   │
└─────────────┬───────────────────────────────────────┘
              ↓
┌─────────────────────────────────────────────────────┐
│  LLM Reasoning                                      │
│  "User wants to move" → navigate action             │
│  Output: {"type": "navigate", "target": "kitchen"}  │
└─────────────┬───────────────────────────────────────┘
              ↓
┌─────────────────────────────────────────────────────┐
│  Pydantic Schema Validation                         │
│  ✓ type is "navigate"                              │
│  ✓ target is string "kitchen"                      │
└─────────────┬───────────────────────────────────────┘
              ↓
┌─────────────────────────────────────────────────────┐
│  Code Dispatch                                      │
│  if type == "navigate":                             │
│      robot.navigate(target)                         │
└─────────────┬───────────────────────────────────────┘
              ↓
┌─────────────────────────────────────────────────────┐
│  Robot Execution                                    │
│  1. Look up "kitchen" coordinates: [5.0, 2.0, 0.0]  │
│  2. Move to target                                   │
│  3. Update current position                          │
└─────────────┬───────────────────────────────────────┘
              ↓
         ✅ Done!
    '''
    print(flow_diagram)


def demo_variations():
    """Demonstrate different navigate variants"""

    print("\n\n" + "="*70)
    print("🔬 Variants of navigate")
    print("="*70)

    robot = MockRobot(name="DEMO")

    variations = [
        {
            "name": "Semantic navigation (recommended)",
            "json": {"type": "navigate", "target": "kitchen"},
            "description": "Use semantic labels; LLM understands easily"
        },
        {
            "name": "Chinese semantic",
            "json": {"type": "navigate", "target": "厨房"},
            "description": "Supports Chinese place names"
        },
        {
            "name": "Coordinate navigation",
            "json": {"type": "navigate", "target": [1.5, 2.3, 0.0]},
            "description": "Direct coordinates (precise but harder for LLM to generate)"
        },
    ]

    for i, var in enumerate(variations, 1):
        print(f"\nVariant {i}: {var['name']}")
        print(f"  Description: {var['description']}")
        print(f"  JSON: {json.dumps(var['json'], ensure_ascii=False)}")

        try:
            action = NavigateAction(**var['json'])
            print(f"  ✅ Schema validation passed")

            # Simulate execution (no real movement)
            target = action.target
            if isinstance(target, str):
                if target in robot.known_locations:
                    coords = robot.known_locations[target]
                    print(f"  🎯 Target coordinates: {coords}")
                else:
                    print(f"  ⚠️  Unknown location: {target}")
            else:
                print(f"  🎯 Direct coordinates: {target}")

        except Exception as e:
            print(f"  ❌ Error: {e}")


def demo_invalid_cases():
    """Demonstrate handling of invalid inputs"""

    print("\n\n" + "="*70)
    print("❌ Invalid input examples (caught by Schema)")
    print("="*70)

    invalid_cases = [
        {
            "name": "Missing target parameter",
            "json": {"type": "navigate"},
            "expected_error": "field required"
        },
        {
            "name": "Incorrect type",
            "json": {"type": "fly", "target": "sky"},
            "expected_error": "Input should be 'navigate'"
        },
        {
            "name": "Coordinates not 3 elements",
            "json": {"type": "navigate", "target": [1.0, 2.0]},
            "expected_error": "Coordinates must be [x, y, z]"
        },
        {
            "name": "Coordinates contain a string",
            "json": {"type": "navigate", "target": [1, 2, "three"]},
            "expected_error": "Input should be a valid number"
        },
    ]

    for i, case in enumerate(invalid_cases, 1):
        print(f"\nCase {i}: {case['name']}")
        print(f"  Input: {json.dumps(case['json'], ensure_ascii=False)}")

        try:
            action = parse_action(case['json'])
            print(f"  ⚠️  Unexpected: validation passed (should not happen)")
        except Exception as e:
            error_msg = str(e)
            print(f"  ✅ Correctly caught: {error_msg[:60]}...")
