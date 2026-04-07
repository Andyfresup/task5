"""
Robot Controller

Integrates brain (LLM) and body (Robot), implements complete perception-decision-execution loop
"""

from typing import List, Dict, Optional
from config import Config
from modules.planning.llm_client import LLMClient
from modules.planning.prompts import get_system_prompt
from modules.planning.schemas import RobotDecision, RobotAction
from modules.execution.mock_robot import MockRobot
from modules.execution.robot_interface import RobotInterface, RobotState


class RobotController:
    """
    Main robot controller

    Responsibilities:
    1. Receive user input
    2. Call LLM for decision making
    3. Execute actions
    4. Manage conversation history
    """

    def __init__(
        self,
        robot: Optional[RobotInterface] = None,
        llm_client: Optional[LLMClient] = None,
        prompt_mode: str = "default",
        show_thought: bool = True
    ):
        """
        Initialize controller

        Args:
            robot: Robot instance (creates MockRobot if None)
            llm_client: LLM client (creates default client if None)
            prompt_mode: Prompt mode (default/simple/debug)
            show_thought: Whether to show LLM thinking process (default True)
        """
        # Initialize robot
        self.robot = robot or MockRobot(name=Config.ROBOT_NAME)

        # Initialize LLM client
        self.llm_client = llm_client or LLMClient()

        # System prompt
        self.system_prompt = get_system_prompt(prompt_mode)

        # Conversation history
        self.conversation_history: List[Dict[str, str]] = []

        # Display options
        self.show_thought = show_thought  # ← Save parameter

        # Statistics
        self.total_interactions = 0
        self.successful_actions = 0
        self.failed_actions = 0

        print(f"\n{'='*60}")
        print(f"🚀 Robot controller initialized successfully")
        print(f"{'='*60}")

    def process_input(self, user_input: str) -> RobotDecision:
        """
        Process user input (complete flow)

        Args:
            user_input: User input text

        Returns:
            RobotDecision: Decision object
        """
        print(f"\n{'='*60}")
        print(f"👤 User: {user_input}")
        print(f"{'='*60}")

        self.total_interactions += 1

        # 1. Let LLM think
        self.robot.set_state(RobotState.THINKING)
        print(f"\n🧠 [Brain thinking...]")

        try:
            decision = self.llm_client.get_decision(
                user_input=user_input,
                system_prompt=self.system_prompt,
                conversation_history=self.conversation_history
            )

            # Print decision
            if self.show_thought and decision.thought:  # ← Check if exists and needs display
                print(f"\n💭 Thinking process: {decision.thought}")
            if decision.reply:
                print(f"💬 Reply: {decision.reply}")
            if decision.action:
                print(f"⚡ Planned action: {decision.action.type}")

            # 2. Execute action
            if decision.action:
                success = self._execute_action(decision.action)
                if success:
                    self.successful_actions += 1
                else:
                    self.failed_actions += 1

            # 3. Update conversation history
            self.conversation_history.append({
                "role": "user",
                "content": user_input
            })

            # Build assistant response (including thinking, reply and action)
            assistant_response_parts = []
            if decision.thought:  # ← Only add when thought exists
                assistant_response_parts.append(f"Thinking: {decision.thought}")
            if decision.reply:
                assistant_response_parts.append(f"Reply: {decision.reply}")
            if decision.action:
                assistant_response_parts.append(f"Action: {decision.action.model_dump_json()}")

            assistant_response = "\n".join(assistant_response_parts)

            self.conversation_history.append({
                "role": "assistant",
                "content": assistant_response
            })

            return decision

        except Exception as e:
            print(f"\n❌ Error: {e}")
            self.robot.set_state(RobotState.ERROR)
            raise

    def _execute_action(self, action: RobotAction) -> bool:
        """
        Execute specific action

        Args:
            action: Action object

        Returns:
            bool: Whether successful
        """
        action_type = action.type

        try:
            if action_type == "navigate":
                return self.robot.navigate(action.target)

            elif action_type == "search":
                result = self.robot.search(action.object_name)
                return result is not None

            elif action_type == "pick":
                return self.robot.pick(action.object_name, action.object_id)

            elif action_type == "place":
                return self.robot.place(action.location)

            elif action_type == "speak":
                return self.robot.speak(action.content)

            elif action_type == "wait":
                return self.robot.wait(action.reason)

            else:
                print(f"⚠ Unknown action type: {action_type}")
                return False

        except Exception as e:
            print(f"❌ Action execution failed: {e}")
            return False

    def interactive_mode(self):
        """
        Interactive mode (command-line conversation)
        """
        print(f"\n{'='*60}")
        print(f"🤖 Entering interactive mode")
        print(f"Tip: type 'quit' or 'exit' to leave")
        print(f"{'='*60}\n")

        while True:
            try:
                user_input = input("👤 You: ").strip()

                if not user_input:
                    continue

                if user_input.lower() in ['quit', 'exit', 'q']:
                    print("\n👋 Goodbye!")
                    self.print_statistics()
                    break

                if user_input.lower() == 'status':
                    self.robot.print_status()
                    continue

                if user_input.lower() == 'stats':
                    self.print_statistics()
                    continue

                # Handle input
                self.process_input(user_input)

            except KeyboardInterrupt:
                print("\n\n👋 Goodbye!")
                self.print_statistics()
                break

            except Exception as e:
                print(f"\n❌ An error occurred: {e}")
                import traceback
                traceback.print_exc()

    def run_test_scenario(self, scenarios: List[str]):
        """
        Run test scenarios

        Args:
            scenarios: List of test cases
        """
        print(f"\n{'='*60}")
        print(f"🧪 Starting test scenarios (total {len(scenarios)})")
        print(f"{'='*60}")

        for i, scenario in enumerate(scenarios, 1):
            print(f"\n\n{'─'*60}")
            print(f"Test {i}/{len(scenarios)}")
            print(f"{'─'*60}")

            try:
                self.process_input(scenario)
                # Wait a bit to simulate real interaction
                import time
                time.sleep(1)

            except Exception as e:
                print(f"❌ Test failed: {e}")

        print(f"\n\n{'='*60}")
        print(f"🏁 Testing complete")
        print(f"{'='*60}")
        self.print_statistics()

    def print_statistics(self):
        """Print statistics"""
        print(f"\n📊 Statistics:")
        print(f"   Total interactions: {self.total_interactions}")
        print(f"   Successful actions: {self.successful_actions}")
        print(f"   Failed actions: {self.failed_actions}")
        if self.total_interactions > 0:
            success_rate = (self.successful_actions / self.total_interactions) * 100
            print(f"   Success rate: {success_rate:.1f}%")

    def reset(self):
        """Reset controller state"""
        self.conversation_history.clear()
        self.total_interactions = 0
        self.successful_actions = 0
        self.failed_actions = 0
        print("✓ Controller reset")


# ==================== Test code ====================

if __name__ == "__main__":
    print("=== Test Robot Controller ===\n")

    # Create controller
    controller = RobotController()

    # Test scenarios
    test_scenarios = [
        "Hello",                          # small talk
        "What's your name?",              # small talk
        "Go to the kitchen",              # simple navigation
        "Help me find an apple",          # search
        "Bring the apple to me",          # compound task
    ]

    # Run tests
    controller.run_test_scenario(test_scenarios)
