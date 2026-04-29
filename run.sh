#!/bin/bash
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=waffle

# Auto-detect the repo root directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Set model path relative to repo
export GZ_SIM_RESOURCE_PATH="${SCRIPT_DIR}/models"

# Launch from repo root
cd "${SCRIPT_DIR}"

# Use command-line argument for world, default to room1.world
WORLD="${1:-room1.world}"

ros2 launch launch/gazebo.launch.py world:="${WORLD}"