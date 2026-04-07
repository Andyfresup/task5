#!/usr/bin/env python3
"""
Demonstrate LLM retry mechanism

Simulate the LLM producing an incorrect output the first time and a correct output on the second attempt.
"""

import sys
import os

# Add project root and src directory to path
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)
sys.path.insert(0, os.path.join(project_root, 'src'))

import json
from modules.planning.schemas import RobotDecision, parse_action


def simulate_llm_retry():
    """Simulate the LLM retry process"""

    print("="*70)
    print("🎬 Scenario: user says 'Go to the table'")
    print("="*70)

    # ==================== First attempt ====================
    print("\n[First attempt]")
    print("─"*70)

    # Possible incorrect output from the LLM on the first try
    response_1 = "Okay, I'll go to the table. I will execute the navigation action immediately."

    print(f"\n📄 LLM raw output:")
    print(f"   {response_1}")

    print(f"\n🔍 Attempting to parse JSON...")

    try:
        # Try to parse
        data = json.loads(response_1)
        print(f"   ✅ Parse successful")
    except json.JSONDecodeError as e:
        print(f"   ❌ Parse failed: {e}")
        print(f"   ⚠ Warning: Could not extract JSON from the text")

        # System will provide feedback to the LLM about the error
        print(f"\n💬 System feedback to LLM:")
        print(f"   'Your output format is incorrect, error: {e}'")
        print(f"   'Please output strictly in JSON format.'")

    # ==================== Second attempt ====================
    print("\n\n[Second attempt]")
    print("─"*70)

    # Correct output from the LLM on the second try
    response_2 = '''```json
{
  "thought": "The user explicitly instructed me to go to the table. This is a direct navigation command. I should perform a navigation action.",
  "reply": "Okay, I'll go to the table now.",
  "action": {
    "type": "navigate",
    "target": "table"
  }
}'''

    print(f"\n📄 LLM raw output:")
    print(response_2)

    print(f"\n🔍 Attempting to parse JSON...")

    try:
        # Extract JSON (supports markdown code blocks)
        if "```json" in response_2:
            start = response_2.find("```json") + 7
            end = response_2.find("```", start)
            json_str = response_2[start:end].strip()
        else:
            json_str = response_2

        # Parse JSON
        data = json.loads(json_str)
        print(f"   ✅ JSON parsed successfully!")

        # Print parsed results
        print(f"\n📋 Parsed results:")
        print(f"   thought: {data['thought'][:50]}...")
        print(f"   reply: {data['reply']}")
        print(f"   action.type: {data['action']['type']}")
        print(f"   action.target: {data['action']['target']}")

        # Validate and create Pydantic model
        action = parse_action(data['action'])
        decision = RobotDecision(**data)

        print(f"\n✅ Pydantic validation passed!")

        # Simulate executing the action
        print(f"\n🤖 Executing action:")
        print(f"   Type: {action.type}")
        print(f"   Target: {action.target}")
        print(f"   → Call robot.navigate('table')")
        print(f"   ✅ Navigation succeeded!")

    except Exception as e:
        print(f"   ❌ Failed: {e}")

    # ==================== Summary ====================
    print("\n\n" + "="*70)
    print("📊 Summary")
    print("="*70)
    print("""
Process:
  1️⃣  User input → "Go to the table"
  2️⃣  LLM 1st attempt → outputs plain text (failed)
  3️⃣  System detects the error → provides feedback to the LLM
  4️⃣  LLM 2nd attempt → outputs correct JSON (success)
  5️⃣  Parse the JSON → extract the action field
  6️⃣  Call the corresponding function → robot.navigate('table')
  7️⃣  Robot executes the action → moves to the table ✅

Key points:
  • JSON is the "communication protocol" between brain and body
  • The action field determines which function to call
  • The retry mechanism ensures success even if the first attempt fails
  • As long as there is one successful attempt, the action will be executed
    """)

    print("="*70)
