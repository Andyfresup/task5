"""
Feedback Collection

"""

from typing import Dict, Any, List
from dataclasses import dataclass
from datetime import datetime


@dataclass
class ExecutionFeedback:
    """
    Execution Feedback Data Structure
    """
    timestamp: float
    action_type: str
    success: bool
    data: Dict[str, Any]
    error_message: str = ""


class FeedbackCollector:
    """
    Feedback Collector (Placeholder)

    TODO: Implement a complete feedback collection mechanism
    - Collect feedback from lower-level modules
    - Detect execution failures
    - Trigger re-planning
    """
    
    def __init__(self):
        self.feedback_history: List[ExecutionFeedback] = []
    
    def collect(self, action_result: Dict[str, Any]) -> ExecutionFeedback:
        """
        Collect execution feedback

        Args:
            action_result: Action execution result

        Returns:
            ExecutionFeedback object
        """
        feedback = ExecutionFeedback(
            timestamp=datetime.now().timestamp(),
            action_type=action_result.get("action", "unknown"),
            success=action_result.get("success", False),
            data=action_result,
            error_message=action_result.get("error", "")
        )
        
        self.feedback_history.append(feedback)
        return feedback
    
    def should_replan(self, feedback: ExecutionFeedback) -> bool:
        """
        Determine whether replanning should be triggered.
        """
        if feedback.success:
            return False

        action = str(feedback.action_type or "").lower()
        if action in ("speak", "wait", "unknown"):
            return False

        payload = feedback.data if isinstance(feedback.data, dict) else {}
        nested = payload.get("data", {}) if isinstance(payload.get("data", {}), dict) else {}
        error_code = str(nested.get("error_code") or "").strip().lower()

        if error_code in (
            "busy_service_workflow",
            "task5_fsm_active",
            "invalid_target",
            "invalid_request",
            "invalid_frame",
            "goal_publish_failed",
            "stale_state",
        ):
            return False

        if error_code in (
            "busy_returning_navigation",
            "ack_timeout",
        ):
            return True

        error_text = str(feedback.error_message or "").lower()
        if "unknown action" in error_text:
            return False

        if "busy_service_workflow" in error_text or "invalid_target" in error_text:
            return False

        return True


# TODO: Future extensions
# - Integrate with Memory Module to store feedback history
# - Integrate with Planning Module to trigger re-planning
# - Implement smarter failure detection and recovery strategies