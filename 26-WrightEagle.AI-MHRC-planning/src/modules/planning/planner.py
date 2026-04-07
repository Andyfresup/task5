"""
Planning Module

"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../..'))

from typing import Optional, List, Dict
from modules.planning.llm_client import LLMClient
from modules.planning.prompts import get_system_prompt
from modules.planning.schemas import RobotDecision


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
        original_decision: RobotDecision
    ) -> RobotDecision:
        """
        Re-plan based on feedback (placeholder)

        TODO: Implement dynamic adjustment logic
        - If an action fails, generate retry or alternative plans
        - Consider environmental changes
        """
        # TODO: Implement replanning logic
        return original_decision


# Predefined action set (matches TDP description)
PREDEFINED_ACTIONS = [
    "navigate",
    "search",
    "pick",
    "place",
    "speak",
    "wait"
]