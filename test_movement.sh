#!/bin/bash
echo "=== Simple Movement Test ==="

# Check if cmd_vel topic exists
if ! ros2 topic list | grep -q "/cmd_vel"; then
    echo "❌ /cmd_vel topic not found. Start your robot simulation first!"
    exit 1
fi

echo "✅ /cmd_vel topic found"

# Test 1: Send forward command
echo "🔄 Testing forward movement..."
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}" --once
sleep 1

# Test 2: Check if command was received
echo "📡 Checking if command was received..."
if timeout 2s ros2 topic echo /cmd_vel --once > /dev/null 2>&1; then
    echo "✅ Command received on /cmd_vel"
else
    echo "❌ No data on /cmd_vel topic"
fi

# Test 3: Stop robot
echo "🛑 Stopping robot..."
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" --once

# Test 4: Check odometry
echo "📍 Checking odometry..."
if timeout 3s ros2 topic echo /odom --once > /dev/null 2>&1; then
    echo "✅ Odometry data available"
    echo "Current position:"
    timeout 2s ros2 topic echo /odom --field pose.pose.position --once
else
    echo "❌ No odometry data"
fi

# Test 5: Check if robot is actually moving
echo "🎯 Testing continuous movement for 3 seconds..."
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}" --rate 5 --times 15 &
PUB_PID=$!

sleep 3
kill $PUB_PID 2>/dev/null

# Stop robot
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}}" --once

echo "✅ Movement test complete!"
echo ""
echo "If you saw the robot moving in Gazebo/RViz, movement is working!"
echo "If not, check:"
echo "1. Is Gazebo running with your robot?"
echo "2. Is robot_state_publisher running?"
echo "3. Are parameter bridges working?"
