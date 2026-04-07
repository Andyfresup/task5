# Team Navigation Repo

This repository contains the navigation stack for the Robocup@Home team.

## Structure

- `src/nav_bringup`: Main launch files and configurations.
- `src/jh_localization`: Custom localization package using LiDAR.
- `src/odom_baselink_tf`: TF publisher for robot transforms.
- `third_party`: External dependencies (e.g., drivers).

## Installation

1. Clone this repository:
   ```bash
   git clone https://github.com/USTC-WrightEagle-AI/navigation.git
   cd navigation
   ```

2. Install dependencies:
   ```bash
   chmod +x install_dependencies.sh
   ./install_dependencies.sh
   ```

3. Build the workspace:
   ```bash
   catkin_make
   ```

4. Source the setup script:
   ```bash
   source devel/setup.bash
   ```

## Usage

To start the navigation stack:

```bash
roslaunch nav_bringup start_navigation.launch
```

## Map

The map is located in `src/nav_bringup/maps/`. You can replace `match_map.yaml` and `match_map.pgm` with your own map.

## Third Party

If you need `livox_ros_driver2`, please add it to `third_party/` or install it separately.
