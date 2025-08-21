#!/bin/bash

# Test script for Vox Nav integration with CropMap V0

echo "=== Vox Nav Integration Test ==="

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# Function to check topic availability
check_topic() {
    local topic=$1
    if ros2 topic list | grep -q "$topic"; then
        echo -e "${GREEN}✓${NC} Topic $topic available"
        return 0
    else
        echo -e "${RED}✗${NC} Topic $topic not found"
        return 1
    fi
}

# Function to check node availability
check_node() {
    local node=$1
    if ros2 node list | grep -q "$node"; then
        echo -e "${GREEN}✓${NC} Node $node running"
        return 0
    else
        echo -e "${RED}✗${NC} Node $node not found"
        return 1
    fi
}

echo "1. Checking Vox Nav nodes..."
check_node "vox_nav_planner_server"
check_node "vox_nav_navigator" 
check_node "vox_nav_map_manager"
check_node "vox_nav_traversability_estimator"

echo ""
echo "2. Checking required topics (using your existing topics)..."
check_topic "/scan"
check_topic "/cmd_vel_nav"
check_topic "/goal_pose"
check_topic "/odometry/filtered"
check_topic "/plan"
check_topic "/local_plan"
check_topic "/octomap_binary"

echo ""
echo "3. Testing Vox Nav planning service..."
if ros2 service list | grep -q "compute_path_to_pose"; then
    echo -e "${GREEN}✓${NC} Vox Nav planning service available"
    
    # Test planning service call
    echo "Testing path planning..."
    ros2 service call /compute_path_to_pose vox_nav_msgs/srv/ComputePathToPose \
        "{start: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}, 
          goal: {position: {x: 2.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}}" &
    
    sleep 5
    echo -e "${BLUE}Check RViz for generated path${NC}"
else
    echo -e "${RED}✗${NC} Vox Nav planning service not found"
fi

echo ""
echo "4. Testing navigation action..."
if ros2 action list | grep -q "navigate_to_pose"; then
    echo -e "${GREEN}✓${NC} Navigation action server available"
    
    # Send navigation goal
    echo "Sending navigation goal..."
    ros2 action send_goal /navigate_to_pose vox_nav_msgs/action/NavigateToPose \
        "{pose: {position: {x: 3.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}" &
    
    echo -e "${BLUE}Check RViz for robot movement${NC}"
else
    echo -e "${RED}✗${NC} Navigation action server not found"
fi

echo ""
echo "5. Monitoring sensor data processing..."
echo "LiDAR data rate:"
ros2 topic hz /livox/lidar --window 10 &
LIDAR_PID=$!

sleep 10
kill $LIDAR_PID 2>/dev/null

echo ""
echo "6. Checking map generation..."
if ros2 topic echo /octomap_binary --once --timeout 5 >/dev/null 2>&1; then
    echo -e "${GREEN}✓${NC} Octomap generation working"
else
    echo -e "${YELLOW}⚠${NC} Octomap generation may need tuning"
fi

echo ""
echo "=== Integration Test Complete ==="
echo "Check RViz for visualization of:"
echo "- Octomap (3D obstacle representation)"
echo "- Elevation map (terrain height data)"
echo "- Planned paths (3D terrain-aware)"
echo "- Robot movement (following 3D paths)"
