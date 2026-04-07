"""
Mock Robot

Simulates robot behavior without real hardware
For PC development and testing
"""

import sys
import os
# Add src directory to Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../..'))

import time
import random
from typing import Optional, List
from modules.execution.robot_interface import RobotInterface, RobotState


class MockRobot(RobotInterface):
    """
    Mock robot class

    Simulates all hardware operations, uses logs instead of real actions
    """

    def __init__(self, name: str = "MockRobot"):
        super().__init__()
        self.name = name
        self.current_position = "home"  # Initial position
        self.holding_object = None      # Currently held object

        # Simulated environment map
        self.known_locations = {
            "home": [0.0, 0.0, 0.0],
            "start_point": [0.0, 0.0, 0.0], 
            "kitchen": [5.0, 2.0, 0.0],
            "living_room": [3.0, -1.0, 0.0],
            "bedroom": [-2.0, 4.0, 0.0],
            "table": [4.0, 1.0, 0.0],
            "desk": [1.0, 3.0, 0.0],
        }

        # Simulated object database
        self.known_objects = {
            "apple": {"name": "apple", "location": "table", "position": [4.0, 1.0, 0.8]},
            "bottle": {"name": "bottle", "location": "kitchen", "position": [5.0, 2.0, 1.0]},
            "cup": {"name": "cup", "location": "table", "position": [4.2, 1.0, 0.8]},
            "book": {"name": "book", "location": "desk", "position": [1.0, 3.0, 0.9]},
        }

        print(f"🤖 {self.name} initialized successfully")
        print(f"   Current position: {self.current_position}")
        print(f"   Known locations: {list(self.known_locations.keys())}")
        print(f"   Known objects: {list(self.known_objects.keys())}")

    def navigate(self, target) -> bool:
        """Simulate navigation"""
        self.set_state(RobotState.EXECUTING)
        print(f"\n🚗 [Navigate] From {self.current_position} to {target}")

        # Simulate movement delay
        time.sleep(0.5)

        # Check if target exists
        if isinstance(target, str):
            if target in self.known_locations:
                self.current_position = target
                coords = self.known_locations[target]
                print(f"✓ Arrived at {target} (coordinates: {coords})")
                self.set_state(RobotState.IDLE)
                return True
            else:
                print(f"✗ Unknown location: {target}")
                self.set_state(RobotState.ERROR)
                return False
        elif isinstance(target, list) and len(target) == 3:
            # Direct coordinate navigation
            self.current_position = f"coordinates{target}"
            print(f"✓ Arrived at coordinates {target}")
            self.set_state(RobotState.IDLE)
            return True
        else:
            print(f"✗ Invalid target format: {target}")
            self.set_state(RobotState.ERROR)
            return False

    def search(self, object_name: str) -> Optional[dict]:
        """Simulate object search"""
        self.set_state(RobotState.EXECUTING)
        print(f"\n🔍 [Search] Searching for: {object_name}")

        # Simulate search delay
        time.sleep(0.8)

        # Look for object
        if object_name in self.known_objects:
            obj = self.known_objects[object_name]
            print(f"✓ Found {object_name} at {obj['location']}")
            print(f"   Position: {obj['position']}")
            self.set_state(RobotState.IDLE)
            return obj
        else:
            # Simulate probability of not finding
            if random.random() < 0.3:
                print(f"✗ Did not find {object_name}")
                self.set_state(RobotState.IDLE)
                return None
            else:
                # Create a "discovered" object
                new_obj = {
                    "name": object_name,
                    "location": self.current_position,
                    "position": [
                        random.uniform(-5, 5),
                        random.uniform(-5, 5),
                        random.uniform(0.5, 1.5)
                    ]
                }
                self.known_objects[object_name] = new_obj
                print(f"✓ Found {object_name} at current location")
                self.set_state(RobotState.IDLE)
                return new_obj

    def pick(self, object_name: str, object_id: Optional[int] = None) -> bool:
        """Simulate object picking"""
        self.set_state(RobotState.EXECUTING)
        print(f"\n🤏 [Pick] Attempting to pick: {object_name}")

        # Check if already holding something
        if self.holding_object:
            print(f"✗ Already holding object: {self.holding_object}")
            self.set_state(RobotState.ERROR)
            return False

        # Simulate picking delay
        time.sleep(0.6)

        # Check if object exists
        if object_name in self.known_objects:
            obj = self.known_objects[object_name]
            # Simplified: don't check distance, assume already nearby
            self.holding_object = object_name
            print(f"✓ Successfully picked {object_name}")
            self.set_state(RobotState.IDLE)
            return True
        else:
            print(f"✗ Object does not exist: {object_name}")
            self.set_state(RobotState.ERROR)
            return False

    def place(self, location) -> bool:
        """
        Simulate placing an object
        """
        self.set_state(RobotState.EXECUTING)
        print(f"\n📦 [Place] Placing object at: {location}")

        # Check if holding an object
        if not self.holding_object:
            print(f"✗ No object in hand")
            self.set_state(RobotState.ERROR)
            return False

        # Simulate placement delay
        time.sleep(0.5)

        print(f"✓ Placed {self.holding_object} at {location}")
        # Update object location
        if self.holding_object in self.known_objects:
            self.known_objects[self.holding_object]["location"] = str(location)

        self.holding_object = None
        self.set_state(RobotState.IDLE)
        return True

    def speak(self, content: str) -> bool:
        """
        Simulate speech output
        """
        self.set_state(RobotState.EXECUTING)
        print(f"\n💬 [Speech] {self.name}: \"{content}\"")

        # Simulate TTS delay
        time.sleep(0.3)

        self.set_state(RobotState.IDLE)
        return True

    def wait(self, reason: Optional[str] = None) -> bool:
        """Wait / no operation"""
        msg = f"\n⏸️  [Wait]"
        if reason:
            msg += f" Reason: {reason}"
        print(msg)

        self.set_state(RobotState.IDLE)
        return True

    def get_status(self) -> dict:
        """Get robot status"""
        return {
            "name": self.name,
            "state": self.state.value,
            "position": self.current_position,
            "holding": self.holding_object,
        }

    def print_status(self):
        """Print current status"""
        status = self.get_status()
        print(f"\n📊 Robot status:")
        print(f"   State: {status['state']}")
        print(f"   Position: {status['position']}")
        print(f"   Holding: {status['holding'] or 'None'}")


# ==================== Test code ====================

if __name__ == "__main__":
    print("=== Test Mock Robot ===\n")

    # Create robot
    robot = MockRobot(name="LARA")

    # Test navigation
    robot.navigate("kitchen")
    robot.print_status()

    # Test search
    result = robot.search("bottle")
    print(f"Search result: {result}")

    # Test pick
    robot.pick("bottle")
    robot.print_status()

    # Test navigate + place
    robot.navigate("table")
    robot.place("table")
    robot.print_status()

    # Test speech
    robot.speak("Task completed!")

    print("\n✓ All tests passed!")
