# cropmap_navigation

This package provides navigation configuration and launch files for running ROS 2 Navigation (Nav2) on crop field or similar environments, with support for keepout zones and SLAM.

## Features

- Launch files for standard navigation and SLAM navigation
- Keepout filter support for costmaps
- Example maps and parameter files
- RViz2 visualization configurations

## Directory Structure

- `launch/`  
  Launch files for navigation, SLAM, and keepout filter servers.
- `config/`  
  Parameter YAML files for navigation, SLAM, and keepout filter nodes.
- `maps/`  
  Example map images and YAML metadata.
- `rviz2/`  
  RViz2 configuration files for visualization.

## Usage

### Prerequisites

- ROS 2 (tested with Humble)

### Build

From your ROS 2 workspace root:

```sh
colcon build --packages-select cropmap_navigation
```

### Launch Navigation

```sh
ros2 launch cropmap_nav2 cropmap_navigation.launch.py
```

This will:
- Start Nav2 with the default map and parameters
- Launch RViz2 with the provided configuration
- Start the keepout filter servers

### Launch SLAM Navigation

```sh
ros2 launch cropmap_nav2 cropmap_navigation_slam.launch.py
```

This will:
- Start SLAM Toolbox for 2d mapping
- Launch Nav2 and RViz2

### Customization

- To use a different map, pass the `map` argument:
  ```sh
  ros2 launch cropmap_navigation cropmap_navigation.launch.py map:=/path/to/map.yaml
  ```
- To use a different parameter file, pass the `params_file` argument.

### Keepout Filter

The keepout filter is configured via the `keepout_fliter.launch.py` launch file and associated YAMLs.  
Edit `params/keepout_fliter_params.yaml` and the referenced mask YAML/image to define keepout zones.

## License

See [package.xml](package.xml) for license details.

## Support

- [NVIDIA Omniverse Isaac Sim Documentation](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/overview.html)
- [NVIDIA Omniverse Forums](https://forums.developer.nvidia.com/c/omniverse/simulation/69)
