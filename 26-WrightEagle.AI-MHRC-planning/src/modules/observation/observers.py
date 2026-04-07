"""
Concrete observer implementations (placeholders)

Current stage: provide basic implementations, future extensions
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../..'))

from datetime import datetime
from modules.observation.observer_interface import Observer, Observation
from typing import Optional


class UserInputObserver(Observer):
    """User input observer"""
    
    def __init__(self):
        self.pending_input: Optional[str] = None
    
    def set_input(self, text: str):
        """Set user input"""
        self.pending_input = text
    
    def collect(self) -> Optional[Observation]:
        """Collect user input"""
        if self.pending_input is None:
            return None
        
        obs = Observation(
            timestamp=datetime.now().timestamp(),
            source="user_input",
            type="text",
            data={"text": self.pending_input}
        )
        self.pending_input = None
        return obs


# TODO: future implementation
class NavigationObserver(Observer):
    """Navigation state observer (placeholder)"""
    def collect(self) -> Optional[Observation]:
        # TODO: subscribe to ROS2 topics like /odom, /map
        return None


class PerceptionObserver(Observer):
    """Perception observer (placeholder)"""
    def collect(self) -> Optional[Observation]:
        # TODO: receive vision detection results, speech recognition results
        return None


class ManipulationObserver(Observer):
    """Manipulation observer (placeholder)"""
    def collect(self) -> Optional[Observation]:
        # TODO: monitor gripper state, joint angles
        return None