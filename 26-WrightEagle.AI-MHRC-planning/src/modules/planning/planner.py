"""
Planning Module

"""

import json
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../..'))

from typing import Optional, List, Dict
from modules.planning.llm_client import LLMClient
from modules.planning.prompts import get_system_prompt
from modules.planning.schemas import RobotDecision, SpeakAction


class Planner:
    """
    Planner

    Use a large language model to decompose natural language instructions into predefined action sequences:
    - navigation
    - grasping
    - placement
    - search
    - speak
    """
    
    def __init__(
        self,
        llm_client: Optional[LLMClient] = None,
        prompt_mode: str = "default"
    ):
        self.llm_client = llm_client or LLMClient()
        self.system_prompt = get_system_prompt(prompt_mode)
    
    def plan(
        self,
        user_input: str,
        context: Optional[Dict] = None
    ) -> RobotDecision:
        """
        Planning decision

        Args:
            user_input: User natural language instruction
            context: Context information (from Memory Module)

        Returns:
            RobotDecision: Contains reasoning, response, and actions
        """
        # Get conversation history from context
        conversation_history = context.get("conversation_history", []) if context else []
        
        # Call LLM to make a decision
        decision = self.llm_client.get_decision(
            user_input=user_input,
            system_prompt=self.system_prompt,
            conversation_history=conversation_history
        )
        
        return decision
    
    def replan(
        self,
        feedback: Dict,
        original_decision: RobotDecision,
        context: Optional[Dict] = None,
    ) -> RobotDecision:
        """
        Re-plan based on action feedback and original decision.
        """
        conversation_history = context.get("conversation_history", []) if context else []

        original_payload = original_decision.model_dump(mode="json")
        feedback_payload = feedback if isinstance(feedback, dict) else {"raw": str(feedback)}

        user_input = (
            "Previous action execution failed. "
            "Generate exactly one safe next action (or speak/wait) in strict JSON.\n"
            "- Do not repeat the same action if feedback indicates hard failure.\n"
            "- If environment/hardware is unavailable, choose speak with a concise status.\n"
            "- Respect the existing action schema.\n"
            "Original decision:\n"
            + json.dumps(original_payload, ensure_ascii=False)
            + "\nExecution feedback:\n"
            + json.dumps(feedback_payload, ensure_ascii=False)
        )

        try:
            return self.llm_client.get_decision(
                user_input=user_input,
                system_prompt=self.system_prompt,
                conversation_history=conversation_history,
            )
        except Exception as exc:
            # Fallback to a deterministic user-facing response if replanning fails.
            return RobotDecision(
                thought="Replanning failed; fallback to explicit status notification.",
                reply=f"I could not recover automatically: {exc}",
                action=SpeakAction(content="I cannot complete this step right now. Please check hardware and retry."),
            )


# Predefined action set (matches TDP description)
PREDEFINED_ACTIONS = [
    "navigate",
    "search",
    "pick",
    "place",
    "speak",
    "wait"
]