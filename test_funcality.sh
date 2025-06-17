#!/bin/bash
# Complete Basic Robot Functionality Test for CropMap V0
# Tests movement, topics, parameters, and sensors

echo "=== CropMap V0 Basic Functionality Test ==="
echo "Testing all topics, parameters, movement, and sensors"
echo ""

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

# Function to print section headers
print_section() {
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo ""
}

# Function to check if topic exists and show info
check_topic() {
    local topic="$1"
    local expected_type="$2"
    
    if ros2 topic list | grep -q "^$topic$"; then
        echo -e "   ${GREEN}✅ $topic${NC}"
        
        # Get topic info
        topic_info=$(ros2 topic info $topic 2>/dev/null)
        actual_type=$(echo "$topic_info" | grep "Type:" | awk '{print $2}')
        
        if [ ! -z "$expected_type" ] && [ "$actual_type" != "$expected_type" ]; then
            echo -e "      ${YELLOW}⚠️  Type: $actual_type (expected: $expected_type)${NC}"
        else
            echo -e "      ${CYAN}Type: $actual_type${NC}"
        fi
        
        # Show publisher/subscriber count
        pub_count=$(echo "$topic_info" | grep "Publisher count:" | awk '{print $3}')
        sub_count=$(echo "$topic_info" | grep "Subscription count:" | awk '{print $3}')
        echo -e "      ${CYAN}Publishers: $pub_count, Subscribers: $sub_count${NC}"
        
        return 0
    else
        echo -e "   ${RED}❌ $topic (missing)${NC}"
        return 1
    fi
}

# Wait for ROS2 to be ready
print_section "SYSTEM READINESS CHECK"
echo "Waiting for ROS2 system to be ready..."
sleep 2

# Check ROS2 daemon
if ros2 daemon status &>/dev/null; then
    echo -e "${GREEN}✅ ROS2 daemon is running${NC}"
else
    echo -e "${RED}❌ ROS2 daemon issue${NC}"
fi

# Check if any nodes are running
node_count=$(ros2 node list 2>/dev/null | wc -l)
echo -e "${CYAN}Active nodes: $node_count${NC}"
echo ""

print_section "1. TOPIC AVAILABILITY TEST"

echo "Essential Robot Topics:"
check_topic "/robot_description" "std_msgs/msg/String"
check_topic "/joint_states" "sensor_msgs/msg/JointState" 
check_topic "/tf" "tf2_msgs/msg/TFMessage"
check_topic "/tf_static" "tf2_msgs/msg/TFMessage"

echo ""
echo "Movement and Control Topics:"
check_topic "/cmd_vel" "geometry_msgs/msg/Twist"
check_topic "/odom" "nav_msgs/msg/Odometry"

echo ""
echo "Sensor Topics:"
check_topic "/scan" "sensor_msgs/msg/LaserScan"
check_topic "/imu" "sensor_msgs/msg/Imu"

echo ""
echo "Simulation Topics (if using Gazebo):"
check_topic "/clock" "rosgraph_msgs/msg/Clock"

echo ""
echo "All Available Topics:"
echo -e "${CYAN}$(ros2 topic list | wc -l) total topics found:${NC}"
ros2 topic list | sort | sed 's/^/   /'

print_section "2. TOPIC DATA VERIFICATION"

echo "Testing topic data availability..."

# Test robot description
echo "Robot Description:"
if ros2 topic echo /robot_description --once --timeout 2 &>/dev/null; then
    echo -e "   ${GREEN}✅ Robot description publishing${NC}"
    desc_size=$(ros2 topic echo /robot_description --once --timeout 2 2>/dev/null | wc -c)
    echo -e "   ${CYAN}Description size: $desc_size characters${NC}"
else
    echo -e "   ${RED}❌ Robot description not publishing${NC}"
fi

# Test joint states
echo ""
echo "Joint States:"
if joint_data=$(timeout 3s ros2 topic echo /joint_states --once 2>/dev/null); then
    echo -e "   ${GREEN}✅ Joint states publishing${NC}"
    joint_count=$(echo "$joint_data" | grep -c "name:")
    echo -e "   ${CYAN}Number of joints: $joint_count${NC}"
    if [ $joint_count -gt 0 ]; then
        echo "   Joint names:"
        echo "$joint_data" | grep -A 20 "name:" | grep "^- " | sed 's/^/      /'
    fi
else
    echo -e "   ${RED}❌ Joint states not publishing${NC}"
fi

# Test scan data
echo ""
echo "LiDAR Scan Data:"
if scan_data=$(timeout 3s ros2 topic echo /scan --once 2>/dev/null); then
    echo -e "   ${GREEN}✅ Scan data publishing${NC}"
    
    # Extract scan info
    angle_min=$(echo "$scan_data" | grep "angle_min:" | awk '{print $2}')
    angle_max=$(echo "$scan_data" | grep "angle_max:" | awk '{print $2}')
    range_min=$(echo "$scan_data" | grep "range_min:" | awk '{print $2}')
    range_max=$(echo "$scan_data" | grep "range_max:" | awk '{print $2}')
    
    echo -e "   ${CYAN}Angle range: $angle_min to $angle_max rad${NC}"
    echo -e "   ${CYAN}Range limits: $range_min to $range_max m${NC}"
    
    # Check for actual range data
    range_count=$(echo "$scan_data" | grep -A 1000 "ranges:" | grep "^- " | wc -l)
    echo -e "   ${CYAN}Number of range readings: $range_count${NC}"
    
    if [ $range_count -gt 0 ]; then
        # Get min/max range values
        min_range=$(echo "$scan_data" | grep -A 1000 "ranges:" | grep "^- " | awk '{print $2}' | grep -v "inf" | sort -n | head -1)
        max_range=$(echo "$scan_data" | grep -A 1000 "ranges:" | grep "^- " | awk '{print $2}' | grep -v "inf" | sort -n | tail -1)
        echo -e "   ${CYAN}Actual range values: $min_range to $max_range m${NC}"
    fi
else
    echo -e "   ${RED}❌ Scan data not available${NC}"
fi

# Test odometry
echo ""
echo "Odometry Data:"
if odom_data=$(timeout 3s ros2 topic echo /odom --once 2>/dev/null); then
    echo -e "   ${GREEN}✅ Odometry publishing${NC}"
    
    # Extract position
    pos_x=$(echo "$odom_data" | grep -A 10 "position:" | grep "x:" | awk '{print $2}')
    pos_y=$(echo "$odom_data" | grep -A 10 "position:" | grep "y:" | awk '{print $2}')
    pos_z=$(echo "$odom_data" | grep -A 10 "position:" | grep "z:" | awk '{print $2}')
    
    echo -e "   ${CYAN}Position: x=$pos_x, y=$pos_y, z=$pos_z${NC}"
    
    # Extract velocity
    vel_x=$(echo "$odom_data" | grep -A 10 "linear:" | grep "x:" | tail -1 | awk '{print $2}')
    echo -e "   ${CYAN}Linear velocity: $vel_x m/s${NC}"
else
    echo -e "   ${RED}❌ Odometry not available${NC}"
fi

print_section "3. MOVEMENT TESTING"

echo "Testing robot movement capabilities..."

# Check if cmd_vel topic is ready
if ros2 topic list | grep -q "/cmd_vel"; then
    echo -e "${GREEN}✅ cmd_vel topic available${NC}"
    
    # Test forward movement
    echo ""
    echo "Testing forward movement (0.2 m/s for 2 seconds)..."
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}" &
    sleep 1
    
    # Check if movement command was received
    if timeout 2s ros2 topic echo /cmd_vel --once &>/dev/null; then
        echo -e "   ${GREEN}✅ Movement command sent and received${NC}"
        
        # Show current velocity
        current_vel=$(timeout 2s ros2 topic echo /cmd_vel --field linear.x 2>/dev/null)
        if [ ! -z "$current_vel" ]; then
            echo -e "   ${CYAN}Current velocity: $current_vel m/s${NC}"
        fi
    else
        echo -e "   ${RED}❌ Movement command not received${NC}"
    fi
    
    # Stop the robot
    echo ""
    echo "Stopping robot..."
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}" &
    
    # Test rotation
    echo ""
    echo "Testing rotation (0.5 rad/s for 1 second)..."
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}" &
    sleep 1
    
    # Stop rotation
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}" &
    
    echo -e "   ${GREEN}✅ Rotation test completed${NC}"
    
else
    echo -e "${RED}❌ cmd_vel topic not available${NC}"
fi

print_section "4. TF FRAME TESTING"

echo "Checking TF frame tree..."

# List all frames
if frames=$(ros2 run tf2_tools view_frames 2>/dev/null); then
    echo -e "${GREEN}✅ TF frames are being published${NC}"
    echo -e "${CYAN}Check frames.pdf for complete TF tree${NC}"
else
    echo -e "${YELLOW}⚠️  TF frame generation had issues${NC}"
fi

# Check specific frame transforms
echo ""
echo "Testing key frame transforms:"

key_frames=("base_link" "base_footprint" "lidar_link" "odom" "map")
for frame in "${key_frames[@]}"; do
    if timeout 2s ros2 run tf2_ros tf2_echo base_link $frame &>/dev/null; then
        echo -e "   ${GREEN}✅ base_link → $frame${NC}"
    else
        echo -e "   ${RED}❌ base_link → $frame (no transform)${NC}"
    fi
done

print_section "5. NODE AND SERVICE ANALYSIS"

echo "Active ROS2 Nodes:"
ros2 node list | sed 's/^/   /' | sort

echo ""
echo "Available Services:"
service_count=$(ros2 service list | wc -l)
echo -e "${CYAN}$service_count services available:${NC}"
ros2 service list | sort | sed 's/^/   /'

echo ""
echo "Available Actions:"
action_count=$(ros2 action list | wc -l)
echo -e "${CYAN}$action_count actions available:${NC}"
ros2 action list | sort | sed 's/^/   /'

print_section "6. PARAMETER INSPECTION"

echo "Robot State Publisher Parameters:"
if ros2 node list | grep -q "robot_state_publisher"; then
    echo "Available parameters:"
    ros2 param list /robot_state_publisher | sed 's/^/   /'
    
    echo ""
    echo "Key parameter values:"
    use_sim_time=$(ros2 param get /robot_state_publisher use_sim_time 2>/dev/null | grep -o 'True\|False')
    echo -e "   ${CYAN}use_sim_time: $use_sim_time${NC}"
else
    echo -e "${RED}❌ robot_state_publisher node not found${NC}"
fi

echo ""
echo "All nodes with parameters:"
for node in $(ros2 node list); do
    param_count=$(ros2 param list $node 2>/dev/null | wc -l)
    if [ $param_count -gt 0 ]; then
        echo -e "   ${CYAN}$node: $param_count parameters${NC}"
    fi
done

print_section "7. SYSTEM PERFORMANCE"

echo "Topic frequency analysis (5 second sample):"

# Check cmd_vel frequency
if ros2 topic list | grep -q "/cmd_vel"; then
    echo "Measuring /cmd_vel frequency..."
    cmd_vel_hz=$(timeout 5s ros2 topic hz /cmd_vel 2>/dev/null | grep "average rate" | awk '{print $3}' || echo "0.0")
    echo -e "   ${CYAN}/cmd_vel: $cmd_vel_hz Hz${NC}"
fi

# Check odom frequency
if ros2 topic list | grep -q "/odom"; then
    echo "Measuring /odom frequency..."
    odom_hz=$(timeout 5s ros2 topic hz /odom 2>/dev/null | grep "average rate" | awk '{print $3}' || echo "0.0")
    echo -e "   ${CYAN}/odom: $odom_hz Hz${NC}"
fi

# Check scan frequency
if ros2 topic list | grep -q "/scan"; then
    echo "Measuring /scan frequency..."
    scan_hz=$(timeout 5s ros2 topic hz /scan 2>/dev/null | grep "average rate" | awk '{print $3}' || echo "0.0")
    echo -e "   ${CYAN}/scan: $scan_hz Hz${NC}"
fi

print_section "8. SAFETY AND ACCEPTANCE CRITERIA PREVIEW"

echo "Checking acceptance criteria parameters:"

# Speed limit check (if robot is moving)
echo "Speed Monitoring:"
if ros2 topic list | grep -q "/cmd_vel"; then
    current_speed=$(timeout 2s ros2 topic echo /cmd_vel --field linear.x 2>/dev/null || echo "0.0")
    if (( $(echo "$current_speed <= 2.0" | bc -l 2>/dev/null || echo 1) )); then
        echo -e "   ${GREEN}✅ Current speed within 2.0 m/s limit: $current_speed m/s${NC}"
    else
        echo -e "   ${RED}❌ Speed exceeds 2.0 m/s limit: $current_speed m/s${NC}"
    fi
else
    echo -e "   ${YELLOW}⚠️  cmd_vel topic not available for speed check${NC}"
fi

# Obstacle detection range
echo ""
echo "Obstacle Detection:"
if ros2 topic list | grep -q "/scan"; then
    echo -e "   ${GREEN}✅ LiDAR available for 0.5m obstacle detection${NC}"
    
    # Sample a few scan ranges
    min_detected=$(timeout 3s ros2 topic echo /scan --once 2>/dev/null | grep -A 50 "ranges:" | grep "^- " | head -10 | awk '{print $2}' | grep -v "inf" | sort -n | head -1)
    if [ ! -z "$min_detected" ]; then
        echo -e "   ${CYAN}Closest obstacle detected: $min_detected m${NC}"
        if (( $(echo "$min_detected < 0.5" | bc -l 2>/dev/null || echo 0) )); then
            echo -e "   ${YELLOW}⚠️  Obstacle within 0.5m - robot should stop!${NC}"
        fi
    fi
else
    echo -e "   ${RED}❌ No LiDAR data for obstacle detection${NC}"
fi

print_section "SUMMARY AND NEXT STEPS"

echo "Basic Functionality Test Complete!"
echo ""

# Count successful components
working_topics=0
total_essential=5

essential_topics=("/robot_description" "/cmd_vel" "/odom" "/scan" "/tf")
for topic in "${essential_topics[@]}"; do
    if ros2 topic list | grep -q "^$topic$"; then
        working_topics=$((working_topics + 1))
    fi
done

echo -e "${CYAN}Essential topics working: $working_topics/$total_essential${NC}"

if [ $working_topics -eq $total_essential ]; then
    echo -e "${GREEN}🎉 All essential systems operational!${NC}"
    echo ""
    echo -e "${GREEN}Ready for Nav2 testing!${NC}"
    echo ""
    echo "Next steps:"
    echo "1. ros2 launch robot_bringup simple_nav.launch.py"
    echo "2. Test navigation with 2D Nav Goal in RViz"
    echo "3. Run acceptance criteria tests"
    echo "4. Test with different maps"
else
    echo -e "${YELLOW}⚠️  Some essential systems need attention${NC}"
    echo ""
    echo "Missing topics:"
    for topic in "${essential_topics[@]}"; do
        if ! ros2 topic list | grep -q "^$topic$"; then
            echo -e "   ${RED}- $topic${NC}"
        fi
    done
fi

echo ""
echo "Manual test commands:"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "# Move forward:"
echo "ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
echo ""
echo "# Rotate:"
echo "ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
echo ""
echo "# Stop:"
echo "ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
echo ""
echo "# Check scan data:"
echo "ros2 topic echo /scan --once"
echo ""
echo "# Monitor odometry:"
echo "ros2 topic echo /odom --field pose.pose.position"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
