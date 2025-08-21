#!/usr/bin/env python3
import numpy as np
from PIL import Image
import os

def create_correct_heightmap(name, size=513):  # 2^9 + 1 = 513
    """Create heightmaps with correct size for Gazebo"""
    heightmap = np.zeros((size, size), dtype=np.uint8)
    
    if 'slope' in name:
        for i in range(size):
            heightmap[i, :] = int(i * 255 / size)
    elif 'crater' in name:
        center = size // 2
        for i in range(size):
            for j in range(size):
                dist = np.sqrt((i - center)**2 + (j - center)**2)
                if dist < size // 4:
                    heightmap[i, j] = 255 - int(dist * 255 / (size // 4))
                else:
                    heightmap[i, j] = 128
    elif 'wavy' in name:
        x = np.linspace(0, 4*np.pi, size)
        y = np.linspace(0, 4*np.pi, size)
        X, Y = np.meshgrid(x, y)
        heightmap = ((np.sin(X) * np.sin(Y) + 1) * 127).astype(np.uint8)
    else:
        heightmap.fill(128)
        center = size // 2
        heightmap[center-20:center+20, center-20:center+20] = 200

    return heightmap

# Absolute path to the script's folder
script_dir = os.path.dirname(os.path.abspath(__file__))
worlds_dir = os.path.join(script_dir)

# Create correct heightmaps
terrain_names = [
    'terrain_1_slope', 'terrain_2_crater', 
    'terrain_3_wavy', 'terrain_4_obstacle', 
    'terrain_5_integrated'
]

for terrain in terrain_names:
    heightmap = create_correct_heightmap(terrain)
    filename = os.path.join(worlds_dir, f"{terrain}_heightmap.png")
    Image.fromarray(heightmap, mode='L').save(filename)
    print(f"✅ Created {filename} - Size: {heightmap.shape}")

print("✅ All heightmaps recreated with correct size (513x513)!")

