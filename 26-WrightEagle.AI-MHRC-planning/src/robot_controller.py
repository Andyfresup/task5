"""
Robot Controller

Integrates brain (LLM) and body (Robot), implements complete perception-decision-execution loop
"""

import copy
from typing import List, Dict, Optional
from config import Config
from modules.planning.llm_client import LLMClient
from modules.planning.planner import Planner
from modules.planning.prompts import get_system_prompt
from modules.planning.schemas import RobotDecision, RobotAction
from modules.execution.feedback import FeedbackCollector
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
        if robot is not None:
            self.robot = robot
        else:
            self.robot = self._create_robot_from_config()

        # Initialize LLM client
        self.llm_client = llm_client or LLMClient()
        self.planner = Planner(llm_client=self.llm_client, prompt_mode=prompt_mode)
        self.feedback_collector = FeedbackCollector()

        self.enable_replan_on_failure = bool(getattr(Config, "ENABLE_REPLAN_ON_FAILURE", True))
        self.max_replan_attempts = max(0, int(getattr(Config, "MAX_REPLAN_ATTEMPTS", 1)))

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

    def _create_robot_from_config(self) -> RobotInterface:
        if Config.ENABLE_MOCK:
            return MockRobot(name=Config.ROBOT_NAME)

        try:
            from modules.execution.task5_ros_adapter import Task5ROSAdapter

            print("🔌 Using Task5ROSAdapter (real ROS execution)")
            return Task5ROSAdapter(name=Config.ROBOT_NAME)
        except Exception as exc:
            print(f"⚠ Failed to initialize Task5ROSAdapter, fallback to MockRobot: {exc}")
            return MockRobot(name=Config.ROBOT_NAME)

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
                action_result = self._execute_action(decision.action)
                self.feedback_collector.collect(action_result)

                if action_result.get("success", False):
                    self.successful_actions += 1
                else:
                    self.failed_actions += 1
                    self._attempt_replan_if_needed(decision, action_result)

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

    def _execute_action(self, action: RobotAction) -> Dict[str, object]:
        """
        Execute specific action

        Args:
            action: Action object

        Returns:
            dict: execution result payload
        """
        action_type = action.type
        result: Dict[str, object] = {
            "success": False,
            "action": action_type,
            "error": "",
            "data": {},
        }

        try:
            if action_type == "navigate":
                result["success"] = bool(self.robot.navigate(action.target))
                result["data"] = {"target": action.target}

            elif action_type == "search":
                search_result = self.robot.search(action.object_name)
                result["success"] = search_result is not None
                result["data"] = {
                    "object_name": action.object_name,
                    "result": search_result,
                }

            elif action_type == "pick":
                result["success"] = bool(self.robot.pick(action.object_name, action.object_id))
                result["data"] = {
                    "object_name": action.object_name,
                    "object_id": action.object_id,
                }

            elif action_type == "place":
                result["success"] = bool(self.robot.place(action.location))
                result["data"] = {"location": action.location}

            elif action_type == "speak":
                result["success"] = bool(self.robot.speak(action.content))
                result["data"] = {"content": action.content}

            elif action_type == "wait":
                result["success"] = bool(self.robot.wait(action.reason))
                result["data"] = {"reason": action.reason}

            else:
                print(f"⚠ Unknown action type: {action_type}")
                result["error"] = f"Unknown action type: {action_type}"

        except Exception as e:
            print(f"❌ Action execution failed: {e}")
            result["error"] = str(e)

        if hasattr(self.robot, "get_last_action_result"):
            try:
                robot_result = self.robot.get_last_action_result()
                if isinstance(robot_result, dict) and robot_result.get("action") == action_type:
                    merged = copy.deepcopy(result)
                    merged.update({k: v for k, v in robot_result.items() if k in ("success", "error", "data")})
                    result = merged
            except Exception:
                pass

        if not result.get("success", False) and not result.get("error"):
            result["error"] = "action_failed"

        return result

    def _extract_error_code(self, result: Dict[str, object]) -> str:
        if not isinstance(result, dict):
            return ""

        data = result.get("data", {})
        if not isinstance(data, dict):
            return ""

        return str(data.get("error_code") or "").strip().lower()

    def _attempt_replan_if_needed(self, original_decision: RobotDecision, failed_result: Dict[str, object]):
        if not self.enable_replan_on_failure or self.max_replan_attempts <= 0:
            return

        error_code = self._extract_error_code(failed_result)
        if error_code in (
            "busy_service_workflow",
            "task5_fsm_active",
            "invalid_target",
            "invalid_request",
            "invalid_frame",
            "stale_state",
        ):
            if error_code in ("busy_service_workflow", "task5_fsm_active"):
                try:
                    self.robot.speak("Task5 is busy with an active service workflow. Please wait.")
                except Exception:
                    pass
            return

        if error_code in ("busy_returning_navigation", "ack_timeout"):
            try:
                self.robot.wait(reason=f"retry_after_{error_code}")
            except Exception:
                pass

        feedback = self.feedback_collector.feedback_history[-1] if self.feedback_collector.feedback_history else None
        if feedback is None:
            feedback = self.feedback_collector.collect(failed_result)
        if not self.feedback_collector.should_replan(feedback):
            return

        replan_context = {"conversation_history": self.conversation_history}
        current_decision = original_decision

        for attempt in range(1, self.max_replan_attempts + 1):
            print(f"🔁 Replan attempt {attempt}/{self.max_replan_attempts}")

            replanned = self.planner.replan(
                feedback=failed_result,
                original_decision=current_decision,
                context=replan_context,
            )

            if replanned.reply:
                print(f"💬 Replan reply: {replanned.reply}")
            if replanned.action:
                print(f"⚡ Replan action: {replanned.action.type}")

            if not replanned.action:
                return

            replan_result = self._execute_action(replanned.action)
            self.feedback_collector.collect(replan_result)
            if replan_result.get("success", False):
                self.successful_actions += 1
                print("✅ Replan action succeeded")
                return

            self.failed_actions += 1
            failed_result = replan_result
            current_decision = replanned
            print(f"❌ Replan action failed: {replan_result.get('error', 'unknown_error')}")

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
