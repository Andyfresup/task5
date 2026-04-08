#!/usr/bin/env python3
"""
CADE - Embodied Intelligence Robot Main Program

Run modes:
1. Interactive mode: python main.py
2. Test mode: python main.py --test
3. Demo mode: python main.py --demo
"""

import sys
import argparse
import threading
from robot_controller import RobotController
from config import Config


def print_banner():
    """Print welcome banner"""
    banner = """
╔═══════════════════════════════════════════════════════════╗
║                                                           ║
║   🤖 CADE - Embodied Intelligence Robot System           ║
║   Cognitive Agent for Domestic Environment                ║
║                                                           ║
║   Project: Project LARA                                   ║
║   Version: 0.1.0                                          ║
║                                                           ║
╚═══════════════════════════════════════════════════════════╝
"""
    print(banner)
    print(f"Run mode: {'☁️  Cloud' if Config.is_cloud_mode() else '💻 Local'}")
    print(f"Model: {Config.get_llm_config()['model']}")
    print(f"Robot: {Config.ROBOT_NAME}")
    print(f"Mock mode: {'✓' if Config.ENABLE_MOCK else '✗'}")
    print()


def interactive_mode(args):
    """Interactive mode"""
    print_banner()
    controller = RobotController(
        prompt_mode=args.mode,
        show_thought=not args.no_thought
    )
    controller.interactive_mode()


def test_mode(args):
    """Test mode"""
    print_banner()
    print("🧪 Running test scenarios\n")

    controller = RobotController(
        prompt_mode=args.mode,
        show_thought=not args.no_thought
    )

    # Test cases
    test_cases = [
        # 1. Chitchat tests
        "Hello",
        "What's your name?",
        "What can you do?",

        # 2. Simple navigation
        "Go to the kitchen",
        "Return to start point",

        # 3. Search tasks
        "Help me find an apple",
        "Find a cup",

        # 4. Pick tasks
        "Pick up the apple",

        # 5. Composite tasks
        "Put the apple on the table",

        # 6. Edge cases
        "Order takeout for me",  # Task that cannot be completed
    ]

    controller.run_test_scenario(test_cases)


def demo_mode(args):
    """Demo mode - show a complete service flow"""
    print_banner()
    print("🎬 Demo mode: Showing complete service flow\n")

    controller = RobotController(
        prompt_mode=args.mode,
        show_thought=not args.no_thought
    )

    demo_scenario = [
        "Hello",
        "I want the apple on the table",
        "Go to the table",
        "Find the apple",
        "Pick up the apple",
        "Return to start point",
        "Thank you",
    ]

    print("📋 Demo scenario: User requests to get the apple on the table\n")
    controller.run_test_scenario(demo_scenario)


def ros_input_mode(args):
    """ROS text-input mode: subscribe speech text topic and process it."""
    print_banner()
    print(f"🎙 ROS input mode: subscribing to {args.speech_topic}\n")

    try:
        import rospy
        from std_msgs.msg import String
    except Exception as exc:
        raise RuntimeError(f"ROS dependencies unavailable: {exc}") from exc

    if not rospy.core.is_initialized():
        rospy.init_node("mhrc_speech_text_bridge", anonymous=True, disable_signals=True)

    controller = RobotController(
        prompt_mode=args.mode,
        show_thought=not args.no_thought,
    )

    callback_lock = threading.Lock()

    def _on_text(msg):
        text = str(msg.data or "").strip()
        if not text:
            return

        with callback_lock:
            print(f"\n🎙 Speech text: {text}")
            try:
                controller.process_input(text)
            except Exception as exc:
                print(f"❌ Failed to process speech text: {exc}")

    rospy.Subscriber(args.speech_topic, String, _on_text, queue_size=20)
    print("✅ MHRC speech-topic bridge ready. Press Ctrl+C to exit.")
    rospy.spin()


def main():
    """Main function"""
    parser = argparse.ArgumentParser(
        description="CADE Embodied Intelligence Robot System",
        formatter_class=argparse.RawDescriptionHelpFormatter,
                epilog="""
Examples:
    python main.py              # Interactive mode
    python main.py --test       # Test mode
    python main.py --demo       # Demo mode
    python main.py --ros-input  # ROS speech text mode
    python main.py --mode debug # Use debug prompt

Tips:
    - Please configure API key in config.py before first run
    - Type 'quit' to exit in interactive mode
    - Type 'status' to view robot status
    - Type 'stats' to view statistics
                """
    )

    parser.add_argument(
        '--test',
        action='store_true',
        help='Run test mode'
    )

    parser.add_argument(
        '--demo',
        action='store_true',
        help='Run demo mode'
    )

    parser.add_argument(
        '--ros-input',
        action='store_true',
        help='Subscribe ROS speech text topic and process text input'
    )

    parser.add_argument(
        '--speech-topic',
        type=str,
        default='/person_following/pause_reply_text',
        help='ROS speech text topic used in --ros-input mode'
    )

    parser.add_argument(
        '--mode',
        type=str,
        choices=['default', 'simple', 'compact', 'debug'],
        default='default',
        help='Prompt mode (default=standard, compact=no thinking, debug=debug)'
    )

    parser.add_argument(
        '--no-thought',
        action='store_true',
        help='Do not show LLM thinking process (test pure execution performance)'
    )

    args = parser.parse_args()

    try:
        if args.ros_input:
            ros_input_mode(args)
        elif args.test:
            test_mode(args)
        elif args.demo:
            demo_mode(args)
        else:
            interactive_mode(args)

    except KeyboardInterrupt:
        print("\n\n👋 Program exited")
        sys.exit(0)

    except Exception as e:
        print(f"\n❌ Error occurred: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
