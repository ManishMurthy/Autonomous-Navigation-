#!/bin/bash

# Flat Terrain Vox Nav Test
# Tests basic Vox Nav functionality on field_obstacles.world

echo "=== FLAT TERRAIN VOX NAV TEST ==="

# Prerequisites check
echo "Checking prerequisites..."

# Check if simulation is running
if ! ros2 topic list | grep -q "/scan"; then
    echo "❌ ERROR: Simulation not running or robot not spawned"
    echo "Make sure you have:"
    echo "1. ros2 launch robot_bringup gazebo_sim.launch.py world:=field_obstacles.world"
    echo "2. Robot spawned and sensors active"
    exit 1
fi

echo "✅ Simulation detected"

# Check if Nav2 is running
if ! ros2 node list | grep -q "bt_navigator"; then
    echo "❌ ERROR: Nav2 not running"
    echo "Launch: ros2 launch cropmap_nav2 cropmap_navigation.launch.py"
    exit 1
fi

echo "✅ Nav2 detected"

# Check if Vox Nav is running
if ! ros2 node list | grep -q "vox_nav_planner_server_rclcpp_node"; then
    echo "❌ ERROR: Vox Nav not running"
    echo "Launch: ros2 launch cropmap_nav2 flat_terrain_voxnav.launch.py"
    exit 1
fi

echo "✅ Vox Nav detected"

# Test 1: Check service availability
echo ""
echo "Test 1: Checking Vox Nav services..."
if ros2 service list | grep -q "compute_path_to_pose"; then
    echo "✅ Planning service available"
else
    echo "❌ Planning service not found"
    exit 1
fi

# Test 2: Check action availability  
echo ""
echo "Test 2: Checking navigation actions..."
if ros2 action list | grep -q "navigate_to_pose"; then
    echo "✅ Navigation action available"
else
    echo "❌ Navigation action not found"
fi

# Test 3: Test service interface discovery
echo ""
echo "Test 3: Discovering correct service interface..."
SERVICE_TYPE=$(ros2 service type /compute_path_to_pose)
echo "Service type: $SERVICE_TYPE"

echo ""
echo "Service interface:"
ros2 interface show $SERVICE_TYPE | head -20

# Test 4: Attempt service calls with different formats
echo ""
echo "Test 4: Testing service call formats..."

echo "Format 1: Basic structure"
timeout 5 ros2 service call /compute_path_to_pose $SERVICE_TYPE \
  "{start: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}, \
    goal: {position: {x: 3.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}}" \
  && echo "✅ Basic format worked!" || echo "❌ Basic format failed"

echo ""
echo "Format 2: With pose headers"
timeout 5 ros2 service call /compute_path_to_pose $SERVICE_TYPE \
  "{start: {header: {frame_id: 'map'}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}, \
    goal: {header: {frame_id: 'map'}, pose: {position: {x: 3.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}}}" \
  && echo "✅ Header format worked!" || echo "❌ Header format failed"

# Test 5: Compare with Nav2 goal
echo ""
echo "Test 5: Comparative testing suggestion..."
echo "Nav2 test: Send goal via RViz '2D Nav Goal' button"
echo "Vox Nav test: Use working service call format above"
echo "Compare: Do both systems generate paths to same goal?"

echo ""
echo "=== TEST SUMMARY ==="
echo "✅ Vox Nav integration: BASIC FUNCTIONALITY"
echo "✅ Service availability: CONFIRMED"
echo "🔄 Service interface: TESTING REQUIRED"
echo "📊 Comparison testing: READY"

echo ""
echo "=== NEXT STEPS ==="
echo "1. Get service call working with correct format"
echo "2. Test path planning on flat terrain with obstacles"
echo "3. Compare Nav2 vs Vox Nav path quality"
echo "4. Validate basic navigation before complex terrain"
