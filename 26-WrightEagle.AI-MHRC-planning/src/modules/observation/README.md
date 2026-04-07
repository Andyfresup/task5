# Observation Module

## Current implementation status

✅ **Implemented**:
- Observer interface definitions (`Observer`, `Observation`)
- User input observer (`UserInputObserver`)

⏳ **To be implemented**:
- NavigationObserver: collects navigation data (position, velocity, map)
- PerceptionObserver: collects perception data (vision, audio)
- ManipulationObserver: collects manipulation data (gripper state)

## Interface overview

All observers inherit from the `Observer` base class and output a unified `Observation` object.

## Future extensions

After integrating with ROS2, observers will subscribe to the following topics:
- /odom (navigation)
- /camera/image_raw (vision)
- /gripper_state (gripper)