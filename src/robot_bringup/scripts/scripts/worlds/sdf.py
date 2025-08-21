from PIL import Image

# Load the PNG image
image = Image.open("my_field_map.png").convert("L")
width, height = image.size
resolution = 0.05  # meters per pixel (same as in your map)
threshold = 100  # black pixels under this are considered obstacles

# Create and open the SDF file
with open("generated_obstacles.world", "w") as f:
    f.write("<?xml version='1.0'?>\n<sdf version='1.6'>\n")
    f.write("<world name='default'>\n")

    for y in range(height):
        for x in range(width):
            pixel = image.getpixel((x, y))
            if pixel < threshold:
                gx = x * resolution
                gy = (height - y) * resolution
                f.write(f"""
  <model name='obstacle_{x}_{y}'>
    <static>true</static>
    <link name='link'>
      <pose>{gx} {gy} 0.1 0 0 0</pose>
      <collision name='collision'>
        <geometry>
          <box><size>0.05 0.05 0.2</size></box>
        </geometry>
      </collision>
      <visual name='visual'>
        <geometry>
          <box><size>0.05 0.05 0.2</size></box>
        </geometry>
      </visual>
    </link>
  </model>
""")

    f.write("</world>\n</sdf>")

