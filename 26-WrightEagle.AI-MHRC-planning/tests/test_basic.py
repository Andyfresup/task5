"""
Basic test cases

Test basic functionality of each module
"""

import sys
import os

# Add project root and src directory to path
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)
sys.path.insert(0, os.path.join(project_root, 'src'))


def test_config():
    """Test configuration module"""
    print("=== Testing Config ===")
    from src.config import Config

    config = Config.get_llm_config()
    assert "base_url" in config
    assert "api_key" in config
    assert "model" in config

    print(f"✓ Config module OK")
    print(f"  Mode: {Config.MODE}")
    print(f"  Model: {config['model']}\n")


def test_schemas():
    """Test data schemas"""
    print("=== Testing Schemas ===")
    from modules.planning.schemas import (
        NavigateAction, SearchAction, RobotDecision, parse_action
    )

    # Test navigate action
    nav = NavigateAction(target="kitchen")
    assert nav.type == "navigate"
    assert nav.target == "kitchen"

    # Test search action
    search = SearchAction(object_name="apple")
    assert search.type == "search"
    assert search.object_name == "apple"

    # Test decision
    decision = RobotDecision(
        thought="test thought",
        reply="test reply",
        action=nav
    )
    assert decision.thought == "test thought"

    # Test parsing
    action_dict = {"type": "navigate", "target": "kitchen"}
    action = parse_action(action_dict)
    assert action.type == "navigate"

    print("✓ Schemas OK\n")


def test_mock_robot():
    """Test Mock Robot"""
    print("=== Testing Mock Robot ===")
    from modules.execution.mock_robot import MockRobot
    from modules.execution.robot_interface import RobotState

    robot = MockRobot(name="TestBot")

    # Test navigate
    success = robot.navigate("kitchen")
    assert success is True
    assert robot.current_position == "kitchen"
    assert robot.get_state() == RobotState.IDLE

    # Test search
    result = robot.search("apple")
    assert result is not None
    assert "name" in result

    # Test speech
    success = robot.speak("test speech")
    assert success is True

    print("✓ Mock Robot OK\n")


def test_prompts():
    """Test prompts"""
    print("=== Testing Prompts ===")
    from modules.planning.prompts import get_system_prompt, add_context

    # Test getting prompt
    prompt = get_system_prompt("default")
    assert len(prompt) > 0
    assert "LARA" in prompt or "robot" in prompt

    # Test simple prompt
    simple = get_system_prompt("simple")
    assert len(simple) > 0

    # Test adding context
    enhanced = add_context(prompt, "Current location: kitchen")
    assert "Current location" in enhanced

    print("✓ Prompts module OK\n")


def run_all_tests():
    """Run all tests"""
    print("\n" + "="*60)
    print("🧪 Starting basic tests")
    print("="*60 + "\n")

    tests = [
        test_config,
        test_schemas,
        test_mock_robot,
        test_prompts,
    ]

    failed = []

    for test_func in tests:
        try:
            test_func()
        except Exception as e:
            print(f"✗ {test_func.__name__} failed: {e}\n")
            import traceback
            traceback.print_exc()
            failed.append(test_func.__name__)

    print("="*60)
    if not failed:
        print("✅ All tests passed!")
    else:
        print(f"❌ {len(failed)} tests failed: {', '.join(failed)}")
    print("="*60)

    return len(failed) == 0


if __name__ == "__main__":
    success = run_all_tests()
    sys.exit(0 if success else 1)
