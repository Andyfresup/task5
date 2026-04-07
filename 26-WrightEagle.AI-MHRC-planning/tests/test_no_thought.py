#!/usr/bin/env python3
"""
Test DeepSeek's "no-thought" mode performance

Compare outputs with/without showing the thought process
"""

import sys
import os

# Add project root and src directory to path
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)
sys.path.insert(0, os.path.join(project_root, 'src'))

from robot_controller import RobotController


def test_with_thought():
    """Standard mode - show thought process"""
    print("="*70)
    print("🧠 Standard mode: show thought process")
    print("="*70)

    controller = RobotController(show_thought=True)

    test_inputs = [
        "Hello",
        "Go to the kitchen",
        "Find an apple"
    ]

    for user_input in test_inputs:
        controller.process_input(user_input)
        print()


def test_without_thought():
    """No-thought mode - hide thought process"""
    print("\n\n" + "="*70)
    print("⚡ No-thought mode: hide thought process")
    print("="*70)

    controller = RobotController(show_thought=False)

    test_inputs = [
        "Hello",
        "Go to the kitchen",
        "Find an apple"
    ]

    for user_input in test_inputs:
        controller.process_input(user_input)
        print()


def compare():
    """Comparison notes"""
    print("\n\n" + "="*70)
    print("📊 Comparison notes")
    print("="*70)

    print("""
Standard mode (show_thought=True) output:
  ✅ 💭 Thought: ...
  ✅ 💬 Reply: ...
  ✅ ⚡ Planned action: ...
  ✅ 🤖 Execute action

No-thought mode (show_thought=False) output:
  ❌ 💭 Thought: ... (hidden)
  ✅ 💬 Reply: ...
  ✅ ⚡ Planned action: ...
  ✅ 🤖 Execute action

Notes:
  • The LLM still generates a 'thought' field (per the prompt)
  • It is only hidden from display
  • Suitable for:
    - Production (users don't need to see the agent's internal monologue)
    - Demos (cleaner output)
    - Performance tests (less terminal output)

If you want the LLM to not generate 'thought' at all, you need to modify the schema and prompt (see option 2)
    """)


if __name__ == "__main__":
    import sys

    if len(sys.argv) > 1:
        if sys.argv[1] == "with":
            test_with_thought()
        elif sys.argv[1] == "without":
            test_without_thought()
        else:
            print("Usage: python test_no_thought.py [with|without]")
    else:
        print("🎬 Full comparison test\n")
        compare()
        print("\nExamples:")
        print("  python test_no_thought.py with      # test only standard mode")
        print("  python test_no_thought.py without   # test only no-thought mode")
