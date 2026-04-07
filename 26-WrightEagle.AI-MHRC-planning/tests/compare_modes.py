#!/usr/bin/env python3
"""
Comparison test: standard mode vs compact mode

Test whether DeepSeek really does not generate the `thought` field
"""

import sys
import os

# Add project root and src directory to path
project_root = os.path.normpath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, project_root)
sys.path.insert(0, os.path.join(project_root, 'src'))

import time
from robot_controller import RobotController


def test_mode(mode_name: str, prompt_mode: str, show_thought: bool = True):
    """Test a specified mode"""

    print("="*70)
    print(f"🧪 Testing mode: {mode_name}")
    print(f"   Prompt: {prompt_mode}")
    print(f"   Show thought: {show_thought}")
    print("="*70)

    controller = RobotController(
        prompt_mode=prompt_mode,
        show_thought=show_thought
    )

    test_inputs = [
        "Hello",
        "Go to the kitchen",
        "Find an apple",
    ]

    total_time = 0
    for user_input in test_inputs:
        start = time.time()
        decision = controller.process_input(user_input)
        elapsed = time.time() - start

        total_time += elapsed

        # Check whether thought is generated
        has_thought = decision.thought is not None
        thought_info = "✅ Present" if has_thought else "❌ Absent"

        print(f"\n📊 Result analysis:")
        print(f"   thought field: {thought_info}")
        if has_thought:
            print(f"   thought length: {len(decision.thought)} characters")
        print(f"   Elapsed: {elapsed:.2f}s")
        print()

    print(f"\n⏱️  Total time: {total_time:.2f}s")
    print(f"📊 Average time: {total_time/len(test_inputs):.2f}s per input")

    return total_time


def main():
    """Main function - comparison test"""

    print("\n" + "🎯"*35)
    print("Comparison test: standard mode vs compact mode")
    print("🎯"*35 + "\n")

    # Test 1: standard mode (default)
    time1 = test_mode(
        mode_name="Standard mode (with thought)",
        prompt_mode="default",
        show_thought=True
    )

    input("\nPress Enter to continue to the next test...")

    # Test 2: compact mode (no thought)
    time2 = test_mode(
        mode_name="Compact mode (no thought)",
        prompt_mode="compact",
        show_thought=False
    )

    # Compare results
    print("\n\n" + "="*70)
    print("📊 Comparison results")
    print("="*70)

    print(f"\nStandard mode total time: {time1:.2f}s")
    print(f"Compact mode total time: {time2:.2f}s")
    print(f"Time difference: {abs(time1-time2):.2f}s")

    if time2 < time1:
        speedup = ((time1 - time2) / time1) * 100
        print(f"⚡ Compact mode is faster by {speedup:.1f}%")
    else:
        slowdown = ((time2 - time1) / time1) * 100
        print(f"⚠️  Compact mode is slower by {slowdown:.1f}%")

    print("\n💡 Key observations:")
    print("   1. Does the LLM output in compact mode truly lack the thought field?")
    print("   2. Does compact mode save inference time?")
    print("   3. Are actions executed with the same accuracy in compact mode?")

    print("\n📖 For detailed analysis:")
    print("   - Raw LLM output is in the '📄 LLM raw output' section")
    print("   - thought field status is in the '📊 Result analysis' section")


if __name__ == "__main__":
    import sys

    if len(sys.argv) > 1:
        mode = sys.argv[1]
        if mode == "default":
            test_mode("Standard mode", "default", True)
        elif mode == "compact":
            test_mode("Compact mode", "compact", False)
        else:
            print(f"Unknown mode: {mode}")
            print("Usage: python compare_modes.py [default|compact]")
    else:
        main()
