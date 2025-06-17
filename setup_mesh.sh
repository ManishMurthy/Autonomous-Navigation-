#!/bin/bash
# Save this as setup_mesh_env.sh in your workspace root

# Get workspace path
WORKSPACE_DIR=$(pwd)

# Get package paths
ROBOT_DESC_PATH=$(ros2 pkg prefix robot_description 2>/dev/null)/share/robot_description
ROBOT_BRINGUP_PATH=$(ros2 pkg prefix robot_bringup 2>/dev/null)/share/robot_bringup

# Export Gazebo resource paths
export GZ_SIM_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH:$ROBOT_DESC_PATH:$ROBOT_BRINGUP_PATH"
export IGN_GAZEBO_RESOURCE_PATH="$IGN_GAZEBO_RESOURCE_PATH:$ROBOT_DESC_PATH:$ROBOT_BRINGUP_PATH"

echo "Gazebo resource paths set:"
echo "GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH"
echo "IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH"
