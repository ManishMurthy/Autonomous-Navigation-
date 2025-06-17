#!/bin/bash
# Farm World Test Commands for CropMap V0

echo "=== CropMap V0 Farm World Testing ==="

WORKSPACE="/home/manish/eiratech_ws"
cd $WORKSPACE

echo "1. Building workspace..."
colcon build --packages-select robot_bringup robot_description
source install/setup.bash

echo ""
echo "2. Checking your world file..."
if [ -f "src/robot_bringup/worlds/farm_world.world" ]; then
    echo "✅ Farm world file found"
    echo "World file contents:"
    head -20 src/robot_bringup/worlds/farm_world.world
else
    echo "❌ Farm world file not found"
fi

echo ""
echo "3. Checking your terrain model..."
if [ -f "src/robot_bringup/models/terrain_1/model.sdf" ]; then
    echo "✅ Terrain model found"
    echo "Model info:"
    grep -A 5 "<model name" src/robot_bringup/models/terrain_1/model.sdf
else
    echo "❌ Terrain model not found"
fi

echo ""
echo "4. Available launch files:"
ls -la src/robot_bringup/launch/

echo ""
echo "5. Testing launch options..."

echo ""
echo "=== OPTION 1: Direct Gazebo with Farm World ==="
echo "Command: ign gazebo src/robot_bringup/worlds/farm_world.world"
echo "This loads your farm world directly in Gazebo"

echo ""
echo "=== OPTION 2: Your Launch Files ==="
echo "Command: ros2 launch robot_bringup gazebo_sim.launch.py"
echo "This uses your custom launch file"

echo ""
echo "=== OPTION 3: Launch with Custom World ==="  
echo "Command: ros2 launch robot_bringup gazebo_sim.launch.py world:=src/robot_bringup/worlds/farm_world.world"
echo "This uses your launch file with farm world"

echo ""
echo "=== OPTION 4: Teleop Testing ==="
echo "Command: ros2 launch robot_bringup teleop.launch.py"
echo "This enables manual control for testing"

echo ""
echo "=== RECOMMENDED TEST SEQUENCE ==="
echo "1. Start farm world simulation:"
echo "   ros2 launch robot_bringup gazebo_sim.launch.py"
echo ""
echo "2. In another terminal, test teleop:"
echo "   ros2 launch robot_bringup teleop.launch.py"
echo ""
echo "3. Test movement commands:"
echo "   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \"{linear: {x: 0.5}}\" --rate 1"
echo ""
echo "4. Monitor robot position:"
echo "   ros2 topic echo /odom --field pose.pose.position"
echo ""
echo "5. Check sensors:"
echo "   ros2 topic echo /scan --once"

echo ""
echo "=== FOR NAV2 TESTING ==="
echo "Once basic movement works in farm world:"
echo "1. Create map of farm environment"
echo "2. Test path planning around terrain"
echo "3. Validate acceptance criteria on farm terrain"
echo "4. Test terrain adaptability (your thesis requirement!)"

echo ""
echo "Your farm world is perfect for:"
echo "- Agricultural robot testing"
echo "- Terrain navigation validation"  
echo "- Uneven ground handling"
echo "- Real-world scenario testing"

echo ""
echo "🚜 Ready to test CropMap V0 in farm environment!"
