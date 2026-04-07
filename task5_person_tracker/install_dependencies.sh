#!/bin/bash

echo "Installing tracking dependencies..."

# Check if ROS is installed (assuming Noetic for now based on context, adapt if needed)
if [ -z "$ROS_DISTRO" ]; then
    echo "ROS environment not found. Please source your ROS setup.bash first."
    exit 1
fi

sudo apt-get update
sudo apt-get install -y \
    ros-$ROS_DISTRO-move-base \
    ros-$ROS_DISTRO-map-server \
    ros-$ROS_DISTRO-dwa-local-planner \
    ros-$ROS_DISTRO-global-planner \
    ros-$ROS_DISTRO-pcl-ros \
    ros-$ROS_DISTRO-pcl-conversions \
    ros-$ROS_DISTRO-tf2-ros \
    ros-$ROS_DISTRO-tf2-eigen \
    libpcl-dev

echo "Dependencies installed successfully."