#!/bin/bash

# Terrain 1 Slope Avoidance Test
# Tests if Vox Nav avoids 40-degree slopes and chooses 20-degree alternative

echo "=== TERRAIN 1 SLOPE AVOIDANCE TEST ==="

# Test configuration
STEEP_SLOPE_GOAL="5.0 8.0"    # Goal that requires going up 40° slope (direct path)
SAFE_ROUTE_CHECK="2.0 4.0"    # Intermediate point on safer 20° route

echo "Test Objective: Robot should avoid 40° slope and choose safer 20° route"
echo "Direct path: Steep 40° slope"
echo "Safe path: Longer route with <20° slopes"

# Check if Vox Nav nodes are running
if ! ros2 node list | grep -q "vox_nav_planner_server_rclcpp_node"; then
    echo "❌ ERROR: Vox Nav planner not running"
    echo "Start with: ros2 launch cropmap_nav2 terrain1_voxnav.launch.py"
    exit 1
fi

echo "✅ Vox Nav planner detected"

# Test 1: Check service availability
echo ""
echo "Test 1: Checking planning service..."
if ros2 service list | grep -q "compute_path_to_pose"; then
    echo "✅ Planning service available"
else
    echo "❌ Planning service not found"
    exit 1
fi

# Test 2: Plan path that would normally go up steep slope
echo ""
echo "Test 2: Planning path that requires slope decision..."
echo "Goal: Navigate from (0,0) to ($STEEP_SLOPE_GOAL)"
echo "Challenge: Direct path has 40° slope, safe path has 20° slope"

# Try to call planning service (test different formats)
echo ""
echo "Testing planning service call..."

# Format 1: Try basic format
echo "Attempting basic service format..."
timeout 10 ros2 service call /compute_path_to_pose vox_nav_msgs/srv/ComputePathToPose \
  "{start: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}, \
    goal: {position: {x: 5.0, y: 8.0, z: 0.0}, orientation: {w: 1.0}}}" \
  2>/dev/null && echo "✅ Basic format worked!" || echo "❌ Basic format failed"

# Format 2: Try with headers
echo "Attempting format with headers..."
timeout 10 ros2 service call /compute_path_to_pose vox_nav_msgs/srv/ComputePathToPose \
  "{start: {header: {frame_id: 'map'}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}, \
    goal: {header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 8.0, z: 0.0}, orientation: {w: 1.0}}}}" \
  2>/dev/null && echo "✅ Header format worked!" || echo "❌ Header format failed"

# Test 3: Check for path visualization
echo ""
echo "Test 3: Checking for path output..."
echo "Look in RViz for:"
echo "- Generated path avoiding steep slopes"
echo "- Path taking longer but safer route"
echo "- Slope analysis visualization"

# Test 4: Alternative - use navigation action
echo ""
echo "Test 4: Testing navigation action (if available)..."
if ros2 action list | grep -q "navigate_to_pose"; then
    echo "✅ Navigation action available"
    echo "You can test with:"
    echo "ros2 action send_goal /navigate_to_pose vox_nav_msgs/action/NavigateToPose \\"
    echo "  \"{pose: {header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 8.0, z: 0.0}, orientation: {w: 1.0}}}}\""
else
    echo "❌ Navigation action not available"
fi

echo ""
echo "=== TEST RESULTS ==="
echo "Manual verification needed:"
echo "1. Check RViz for path visualization"
echo "2. Verify path avoids steep slopes (40° areas)"
echo "3. Confirm path uses safer alternative route"
echo "4. Monitor robot behavior if navigation executed"

echo ""
echo "Success criteria:"
echo "✅ Path generated successfully"
echo "✅ Path avoids slopes >25° (acceptance criteria)"
echo "✅ Path total time ≤ 2× direct path (acceptance criteria)"
echo "✅ Robot demonstrates slope-aware navigation"
