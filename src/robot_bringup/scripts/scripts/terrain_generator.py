#!/usr/bin/env python3
"""
Terrain Generator for CropMap V0 Testing
Generates 5 required terrain scenarios for acceptance criteria validation
"""

import os
import numpy as np
from PIL import Image
import yaml

class TerrainGenerator:
    def __init__(self, output_dir="worlds"):
        self.output_dir = output_dir
        self.world_template = """<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="{world_name}">
    <!-- Physics -->
    <physics type="ode">
      <real_time_update_rate>1000.0</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
    </physics>
    
    <!-- Lighting -->
    <include>
      <uri>model://sun</uri>
    </include>
    
    <!-- Ground plane with heightmap -->
    <model name="terrain">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <heightmap>
              <uri>{heightmap_path}</uri>
              <size>50 50 10</size>
              <pos>0 0 0</pos>
            </heightmap>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <heightmap>
              <uri>{heightmap_path}</uri>
              <size>50 50 10</size>
              <pos>0 0 0</pos>
              <texture>
                <diffuse>file://media/materials/textures/grass.png</diffuse>
                <normal>file://media/materials/textures/flat_normal.png</normal>
                <size>10</size>
              </texture>
            </heightmap>
          </geometry>
        </visual>
      </link>
    </model>
    
    {obstacles}
    
    <!-- Robot spawn point -->
    <model name="spawn_point">
      <pose>-20 -20 1 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.5</radius>
              <length>0.1</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0 1 0 1</ambient>
          </material>
        </visual>
      </link>
    </model>
    
    <!-- Goal point -->
    <model name="goal_point">
      <pose>20 20 1 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.5</radius>
              <length>0.1</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>"""

    def generate_heightmap(self, terrain_type, size=512):
        """Generate heightmap for different terrain types"""
        heightmap = np.zeros((size, size), dtype=np.uint8)
        
        if terrain_type == "terrain_1_slope":
            # Terrain 1: Slope test (steep 40° vs gentle 20°)
            x = np.linspace(0, 1, size)
            y = np.linspace(0, 1, size)
            X, Y = np.meshgrid(x, y)
            
            # Create two paths: steep direct, gentle curved
            steep_mask = (Y > 0.3) & (Y < 0.7) & (X > 0.2) & (X < 0.8)
            gentle_path = ((Y - 0.1)**2 + (X - 0.5)**2 < 0.15) | ((Y - 0.9)**2 + (X - 0.5)**2 < 0.15)
            
            # Steep path: 40° slope (tan(40°) ≈ 0.84)
            heightmap[steep_mask] = (X[steep_mask] * 150).astype(np.uint8)
            
            # Gentle path: 20° slope (tan(20°) ≈ 0.36) 
            heightmap[gentle_path] = (X[gentle_path] * 60).astype(np.uint8)
            
        elif terrain_type == "terrain_2_crater":
            # Terrain 2: Crater avoidance
            x = np.linspace(-1, 1, size)
            y = np.linspace(-1, 1, size)
            X, Y = np.meshgrid(x, y)
            
            # Create craters (negative gaussians)
            crater1 = np.exp(-((X-0.2)**2 + (Y-0.3)**2) / 0.05) * 60
            crater2 = np.exp(-((X+0.1)**2 + (Y-0.1)**2) / 0.03) * 45
            crater3 = np.exp(-((X-0.3)**2 + (Y+0.2)**2) / 0.04) * 50
            
            heightmap = (128 - crater1 - crater2 - crater3).clip(0, 255).astype(np.uint8)
            
        elif terrain_type == "terrain_3_wavy":
            # Terrain 3: Wavy terrain with sine waves
            x = np.linspace(0, 4*np.pi, size)
            y = np.linspace(0, 4*np.pi, size)
            X, Y = np.meshgrid(x, y)
            
            # Sine wave pattern (10-15cm amplitude)
            wave_pattern = np.sin(X) * np.sin(Y) * 25 + 128
            heightmap = wave_pattern.clip(0, 255).astype(np.uint8)
            
        elif terrain_type == "terrain_4_obstacle":
            # Terrain 4: Flat with obstacle avoidance test
            heightmap.fill(128)  # Flat terrain
            
        elif terrain_type == "terrain_5_integrated":
            # Terrain 5: Combined all previous terrains
            x = np.linspace(-1, 1, size)
            y = np.linspace(-1, 1, size)
            X, Y = np.meshgrid(x, y)
            
            # Slope section
            slope_mask = X < -0.3
            heightmap[slope_mask] = ((X[slope_mask] + 1) * 100 + 128).clip(0, 255).astype(np.uint8)
            
            # Crater section
            crater_mask = (X > -0.3) & (X < 0.3)
            craters = np.exp(-((X-0.0)**2 + (Y-0.0)**2) / 0.1) * 50
            heightmap[crater_mask] = (128 - craters[crater_mask]).clip(0, 255).astype(np.uint8)
            
            # Wavy section
            wave_mask = X > 0.3
            waves = np.sin(X[wave_mask] * 8) * np.sin(Y[wave_mask] * 8) * 20
            heightmap[wave_mask] = (waves + 128).clip(0, 255).astype(np.uint8)
            
        return heightmap

    def generate_obstacles(self, terrain_type):
        """Generate obstacle definitions for SDF"""
        if terrain_type == "terrain_4_obstacle" or terrain_type == "terrain_5_integrated":
            return """
    <!-- Static obstacles -->
    <model name="large_obstacle">
      <pose>10 5 1 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>3 3 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>3 3 2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.3 0.1 1</ambient>
          </material>
        </visual>
      </link>
    </model>
    
    <model name="small_obstacle_1">
      <pose>5 10 1 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.3 0.3 0.3 1</ambient>
          </material>
        </visual>
      </link>
    </model>
    
    <model name="small_obstacle_2">
      <pose>8 -3 1 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1.5 1.5 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1.5 1.5 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.3 0.3 0.3 1</ambient>
          </material>
        </visual>
      </link>
    </model>
    
    <!-- Dynamic obstacle (moving) -->
    <model name="dynamic_obstacle">
      <pose>0 0 1 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.5</radius>
              <length>1.5</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.5</radius>
              <length>1.5</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
          </material>
        </visual>
        <inertial>
          <mass>10</mass>
          <inertia>
            <ixx>1</ixx>
            <iyy>1</iyy>
            <izz>1</izz>
          </inertia>
        </inertial>
      </link>
      
      <!-- Add plugin to move the obstacle -->
      <plugin name="dynamic_motion" filename="libgazebo_ros_planar_move.so">
        <command_topic>cmd_vel_dynamic</command_topic>
        <odometry_topic>odom_dynamic</odometry_topic>
        <odometry_frame>odom</odometry_frame>
        <robot_base_frame>dynamic_obstacle</robot_base_frame>
        <odometry_rate>20.0</odometry_rate>
      </plugin>
    </model>
            """
        return ""

    def generate_terrain_world(self, terrain_type):
        """Generate complete world file for terrain type"""
        print(f"Generating {terrain_type}...")
        
        # Create heightmap
        heightmap = self.generate_heightmap(terrain_type)
        
        # Save heightmap as PNG
        heightmap_filename = f"{terrain_type}_heightmap.png"
        heightmap_path = os.path.join(self.output_dir, heightmap_filename)
        
        os.makedirs(self.output_dir, exist_ok=True)
        Image.fromarray(heightmap, mode='L').save(heightmap_path)
        
        # Generate obstacles
        obstacles = self.generate_obstacles(terrain_type)
        
        # Create world file
        world_content = self.world_template.format(
            world_name=terrain_type,
            heightmap_path=f"file://{os.path.abspath(heightmap_path)}",
            obstacles=obstacles
        )
        
        world_filename = f"{terrain_type}.world"
        world_path = os.path.join(self.output_dir, world_filename)
        
        with open(world_path, 'w') as f:
            f.write(world_content)
            
        print(f"✅ Created {world_filename} and {heightmap_filename}")
        return world_path

    def generate_all_terrains(self):
        """Generate all 5 required terrain scenarios"""
        terrain_types = [
            "terrain_1_slope",
            "terrain_2_crater", 
            "terrain_3_wavy",
            "terrain_4_obstacle",
            "terrain_5_integrated"
        ]
        
        world_files = []
        for terrain in terrain_types:
            world_file = self.generate_terrain_world(terrain)
            world_files.append(world_file)
            
        # Generate launch files for each terrain
        self.generate_test_launch_files(terrain_types)
        
        return world_files

    def generate_test_launch_files(self, terrain_types):
        """Generate launch files for terrain testing"""
        launch_template = '''#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directories
    robot_bringup_dir = get_package_share_directory('robot_bringup')
    cropmap_nav2_dir = get_package_share_directory('cropmap_nav2')
    
    # World file path
    world_file = os.path.join(robot_bringup_dir, 'worlds', '{world_file}')
    
    return LaunchDescription([
        # Launch Gazebo with terrain
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([robot_bringup_dir, '/launch/gazebo_sim.launch.py']),
            launch_arguments={{'world': world_file}}.items()
        ),
        
        # Launch navigation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([cropmap_nav2_dir, '/launch/cropmap_navigation.launch.py'])
        ),
        
        # Start terrain test node
        Node(
            package='cropmap_nav2',
            executable='terrain_test_node',
            name='terrain_test',
            parameters=[{{'terrain_type': '{terrain_name}'}}]
        )
    ])
'''
        
        for terrain in terrain_types:
            launch_content = launch_template.format(
                world_file=f"{terrain}.world",
                terrain_name=terrain
            )
            
            launch_path = os.path.join(self.output_dir, f"test_{terrain}.launch.py")
            with open(launch_path, 'w') as f:
                f.write(launch_content)
            print(f"✅ Created test_{terrain}.launch.py")

if __name__ == "__main__":
    generator = TerrainGenerator()
    print("🚀 Generating 5 terrain scenarios for CropMap V0 testing...")
    world_files = generator.generate_all_terrains()
    print(f"\n✅ Successfully generated {len(world_files)} terrain worlds!")
    print("\n📁 Generated files:")
    for world_file in world_files:
        print(f"  - {os.path.basename(world_file)}")
    print("\n🎯 Ready for acceptance criteria testing!")
