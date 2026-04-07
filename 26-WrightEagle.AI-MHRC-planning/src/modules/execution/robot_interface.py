"""
Robot Interface

Defines unified robot abstract interface, both Mock and Real classes must implement this interface
"""

from abc import ABC, abstractmethod
from typing import List, Optional
from enum import Enum


class RobotState(str, Enum):
    """Robot state"""
    IDLE = "IDLE"              # Idle
    THINKING = "THINKING"      # Thinking (LLM reasoning)
    EXECUTING = "EXECUTING"    # Executing
    ERROR = "ERROR"            # Error state


class RobotInterface(ABC):
    """
    Robot abstract interface

    All robot classes (Mock/Real) must implement these methods
    """

    def __init__(self):
        self.state = RobotState.IDLE
        self.current_position: Optional[str] = None
        self.holding_object: Optional[str] = None

    @abstractmethod
    def navigate(self, target) -> bool:
        """
        Navigate to target location

        Args:
            target: Target location (semantic label or coordinates)

        Returns:
            bool: Whether successful
        """
        pass

    @abstractmethod
    def search(self, object_name: str) -> Optional[dict]:
        """
        Search for object

        Args:
            object_name: Object name

        Returns:
            dict: Found object information, like {"name": "apple", "position": [x,y,z]}
            None: Not found
        """
        pass

    @abstractmethod
    def pick(self, object_name: str, object_id: Optional[int] = None) -> bool:
        """
        Pick up object

        Args:
            object_name: Object name
            object_id: Object ID (if multiple objects with same name)

        Returns:
            bool: Whether successful
        """
        pass

    @abstractmethod
    def place(self, location) -> bool:
        """
        Place object

        Args:
            location: Place location

        Returns:
            bool: Whether successful
        """
        pass

    @abstractmethod
    def speak(self, content: str) -> bool:
        """
        Voice output

        Args:
            content: Content to speak

        Returns:
            bool: Whether successful
        """
        pass

    @abstractmethod
    def wait(self, reason: Optional[str] = None) -> bool:
        """
        Wait / No-op

        Args:
            reason: Reason for waiting

        Returns:
            bool: Whether successful
        """
        pass

    def get_state(self) -> RobotState:
        """Get current state"""
        return self.state

    def set_state(self, state: RobotState):
        """Set state"""
        self.state = state
