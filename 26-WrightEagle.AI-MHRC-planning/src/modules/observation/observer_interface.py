"""
Observation Module

"""

from abc import ABC, abstractmethod
from typing import Any, Dict, Optional
from dataclasses import dataclass
from datetime import datetime


@dataclass
class Observation:
    """Observation data structure"""
    timestamp: float
    source: str      # navigation/perception/manipulation/user_input
    type: str        # specific type
    data: Dict[str, Any]


class Observer(ABC):
    """Base class for observers"""
    
    @abstractmethod
    def collect(self) -> Optional[Observation]:
        """Collect observation data"""
        pass


# TODO: Implement concrete observers later
# - NavigationObserver: collect navigation information (position, velocity, map)
# - PerceptionObserver: collect perception information (vision, speech)
# - ManipulationObserver: collect manipulation information (grasping state)