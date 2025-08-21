#!/usr/bin/env python3
# occupancy_to_pcd.py

import numpy as np
import yaml
from PIL import Image
import open3d as o3d

def occupancy_grid_to_pcd(yaml_file, pgm_file, output_pcd):
    # Load map metadata
    with open(yaml_file, 'r') as f:
        map_data = yaml.safe_load(f)
    
    resolution = map_data['resolution']
    origin = map_data['origin']
    
    # Load occupancy grid image
    img = Image.open(pgm_file)
    img_array = np.array(img)
    
    # Convert to 3D points
    points = []
    colors = []
    
    height, width = img_array.shape
    for y in range(height):
        for x in range(width):
            pixel_value = img_array[y, x]
            
            # Convert pixel to world coordinates
            world_x = origin[0] + x * resolution
            world_y = origin[1] + (height - y) * resolution
            
            if pixel_value < 50:  # Occupied space (obstacles)
                # Add points at different heights for 3D structure
                for z in [0.0, 0.5, 1.0, 1.5]:  # Create height variation
                    points.append([world_x, world_y, z])
                    colors.append([0.5, 0.5, 0.5])  # Gray for obstacles
            elif pixel_value > 200:  # Free space (ground)
                points.append([world_x, world_y, 0.0])
                colors.append([0.0, 1.0, 0.0])  # Green for ground
    
    # Create Open3D point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.array(points))
    pcd.colors = o3d.utility.Vector3dVector(np.array(colors))
    
    # Save as PCD
    o3d.io.write_point_cloud(output_pcd, pcd)
    print(f"Saved {len(points)} points to {output_pcd}")

# Usage
occupancy_grid_to_pcd(
    "/home/manish/eiratech_ws/src/cropmap_nav2/maps/final_slam_map.yaml",
    "/home/manish/eiratech_ws/src/cropmap_nav2/maps/final_slam_map.pgm", 
    "/home/manish/eiratech_ws/src/vox_nav/vox_nav_map_server/maps/uneven_terrain.pcd"
)
