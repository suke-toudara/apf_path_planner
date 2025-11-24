# APF Path Planner for ROS2

ROS2 C++ implementation of Artificial Potential Field (APF) path planner.

Based on: https://github.com/eevci/APF-Path-Planner

## Overview

This package provides a path planning algorithm using artificial potential fields. The robot is attracted to the goal while being repelled by obstacles, creating a navigation path.

## Features

- Clean, readable C++ implementation
- ROS2 parameter support for runtime configuration
- Force field visualization
- Header-only Vector2D utility class
- Extensible BasePlanner interface

## Dependencies

- ROS2 (Humble/Iron/Rolling)
- nav_msgs
- geometry_msgs
- visualization_msgs
- tf2

## Building

```bash
cd ~/ros2_ws/src
git clone <this-repo>
cd ~/ros2_ws
colcon build --packages-select apf_path_planner
source install/setup.bash
```

## Usage

```bash
ros2 run apf_path_planner apf_path_planner_node
```

## Parameters

- `attraction_gain` (default: 10.0) - Strength of attraction to goal
- `repulsion_gain` (default: 100.0) - Strength of repulsion from obstacles
- `radius` (default: 10.0) - Influence radius for obstacle repulsion
- `forcemap_width` (default: 10) - Width of force field visualization
- `forcemap_height` (default: 10) - Height of force field visualization

## Topics

### Subscribed

- `/map` (nav_msgs/OccupancyGrid) - Occupancy grid map
- `/initialpose` (geometry_msgs/PoseWithCovarianceStamped) - Initial robot pose
- `/goal_pose` (geometry_msgs/PoseStamped) - Goal pose

### Published

- `/path` (nav_msgs/Path) - Planned path
- `/force_arrows` (visualization_msgs/MarkerArray) - Force field visualization

## Implementation Notes

- Maximum iteration limit: 10000
- Obstacle threshold: 50 (occupancy grid values > 50 are considered obstacles)
- Path resolution follows map resolution
