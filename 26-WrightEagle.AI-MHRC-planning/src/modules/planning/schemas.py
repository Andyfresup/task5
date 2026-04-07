"""
Data Schema Definitions

Uses Pydantic to strictly define robot's action space and decision output format
"""

from typing import Literal, Union, Optional, List
from pydantic import BaseModel, Field, field_validator


# ==================== Action Type Definitions ====================

class NavigateAction(BaseModel):
    """Navigate action - Move to specified location"""
    type: Literal["navigate"] = "navigate"
    target: Union[str, List[float]] = Field(
        ...,
        description="Target location, can be semantic label (like 'kitchen') or coordinates [x,y,z]"
    )

    @field_validator('target')
    @classmethod
    def validate_target(cls, v):
        if isinstance(v, list):
            if len(v) != 3:
                raise ValueError("Coordinates must be in [x, y, z] format")
            if not all(isinstance(i, (int, float)) for i in v):
                raise ValueError("Coordinates must be numbers")
        return v


class PickAction(BaseModel):
    """Pick action - Pick up object"""
    type: Literal["pick"] = "pick"
    object_name: str = Field(..., description="Object name")
    object_id: Optional[int] = Field(None, description="Object ID (if multiple objects with same name)")


class PlaceAction(BaseModel):
    """Place action - Place object at specified location"""
    type: Literal["place"] = "place"
    location: Union[str, List[float]] = Field(
        ...,
        description="Place location, can be semantic label (like 'table') or coordinates"
    )


class SearchAction(BaseModel):
    """Search action - Search for object"""
    type: Literal["search"] = "search"
    object_name: str = Field(..., description="Object name to search for")


class SpeakAction(BaseModel):
    """Speak action - Voice output"""
    type: Literal["speak"] = "speak"
    content: str = Field(..., description="Content to speak")


class WaitAction(BaseModel):
    """Wait action - Maintain current state"""
    type: Literal["wait"] = "wait"
    reason: Optional[str] = Field(None, description="Reason for waiting")


# ==================== Union Action Type ====================

# All possible action types
RobotAction = Union[
    NavigateAction,
    PickAction,
    PlaceAction,
    SearchAction,
    SpeakAction,
    WaitAction
]


# ==================== Decision Output Format ====================

class RobotDecision(BaseModel):
    """
    Robot decision output format (LLM output structure)

    This is the output format that LLM must follow, including thinking process, reply and action
    """
    thought: Optional[str] = Field(  # ← Changed to optional
        None,
        description="Internal thinking process (CoT - Chain of Thought)"
    )
    reply: Optional[str] = Field(
        None,
        description="Natural language reply to user (optional)"
    )
    action: Optional[RobotAction] = Field(
        None,
        description="Action to execute (None for pure conversation)"
    )

    class Config:
        # Allow arbitrary types (for handling union types)
        arbitrary_types_allowed = True


# ==================== Helper Functions ====================

def parse_action(action_dict: dict) -> RobotAction:
    """
    Parse action based on type field

    Args:
        action_dict: Dictionary containing type field

    Returns:
        Corresponding Action object

    Raises:
        ValueError: If type is invalid
    """
    action_type = action_dict.get("type")

    action_map = {
        "navigate": NavigateAction,
        "pick": PickAction,
        "place": PlaceAction,
        "search": SearchAction,
        "speak": SpeakAction,
        "wait": WaitAction,
    }

    if action_type not in action_map:
        raise ValueError(
            f"Unknown action type: {action_type}. "
            f"Available actions: {list(action_map.keys())}"
        )

    return action_map[action_type](**action_dict)


# ==================== Sample Data ====================

if __name__ == "__main__":
    # Test cases
    print("=== Testing Action Definitions ===\n")

    # 1. Navigate action
    nav_action = NavigateAction(target="kitchen")
    print(f"1. Navigate (semantic): {nav_action.model_dump_json(indent=2)}\n")

    nav_action_coords = NavigateAction(target=[1.5, 2.3, 0.0])
    print(f"2. Navigate (coordinates): {nav_action_coords.model_dump_json(indent=2)}\n")

    # 2. Full decision
    decision = RobotDecision(
        thought="User wants an apple; I need to locate it first",
        reply="Okay, I'll go find the apple now",
        action=SearchAction(object_name="apple")
    )
    print(f"3. Full decision: {decision.model_dump_json(indent=2)}\n")

    # 3. Pure conversation (no action)
    chat_decision = RobotDecision(
        thought="The user is greeting me",
        reply="Hello! I'm LARA, the service robot. I'm happy to assist you",
        action=None
    )
    print(f"4. Pure conversation: {chat_decision.model_dump_json(indent=2)}\n")

    print("✓ All tests passed!")
