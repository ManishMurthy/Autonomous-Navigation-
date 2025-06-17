#!/bin/bash
# Clean launch script for CropMap V0 - fixes shared memory issues

echo "=== CropMap V0 Clean Launch ==="

# Function to cleanup
cleanup() {
    echo "Cleaning up..."
    pkill -f ros2
    pkill -f gazebo
    pkill -f ign
    sudo rm -rf /dev/shm/fastrtps_* 2>/dev/null
    sudo rm -rf /dev/shm/fast_* 2>/dev/null
}

# Cleanup first
echo "1. Cleaning up previous sessions..."
cleanup

# Set environment to avoid shared memory issues
echo "2. Setting up clean environment..."
export ROS_DISABLE_LOANED_MESSAGES=1
export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp

# Alternative FastDDS settings (comment out the above if you want to use FastDDS)
# export FASTRTPS_DEFAULT_PROFILES_FILE=""
# export RMW_FASTRTPS_USE_QOS_FROM_XML=0

# Set ROS domain to avoid conflicts
export ROS_DOMAIN_ID=42

echo "3. Restarting ROS2 daemon..."
ros2 daemon stop
sleep 2
ros2 daemon start

echo "4. Setting up workspace..."
cd /home/manish/eiratech_ws/
source install/setup.bash

echo "5. Available launch options:"
echo "   a) Basic robot visualization: ros2 launch robot_description view_robot.launch.py"
echo "   b) Gazebo simulation: ros2 launch robot_bringup gazebo_sim.launch.py"
echo "   c) GZ simulation: ros2 launch robot_bringup gz_sim.launch.py"

echo ""
echo "Choose option (a/b/c) or press Enter for option a:"
read -r choice

case $choice in
    "b")
        echo "Starting Gazebo simulation..."
        ros2 launch robot_bringup gazebo_sim.launch.py
        ;;
    "c") 
        echo "Starting GZ simulation..."
        ros2 launch robot_bringup gz_sim.launch.py
        ;;
    *)
        echo "Starting basic robot visualization..."
        ros2 launch robot_description view_robot.launch.py
        ;;
esac

# Cleanup on exit
trap cleanup EXIT
