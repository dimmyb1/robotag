# Fabemia2 - Autonomous Delivery Robot

A ROS 2-based autonomous delivery robot with differential drive locomotion and multi-camera vision system for navigation and house detection tasks.

## Overview

Fabemia2 is a mobile robot platform designed for autonomous navigation and delivery missions. The robot features a differential drive base with four strategically positioned RGB cameras for comprehensive environmental perception and navigation capabilities. 

## Hardware Specifications

### Mobile Base
- **Dimensions**: 0.7m × 0.5m × 0.3m (L×W×H)
- **Mass**: 5.0 kg
- **Drive System**: Differential drive with 2 powered rear wheels
- **Wheel Radius**: 0.15 m
- **Wheel Separation**: 0.56 m
- **Support**: 2 frictionless casters (front/rear) 

### Vision System
Four RGB cameras configured for different navigation tasks:

| Camera | Position | Orientation | Purpose |
|--------|----------|-------------|---------|
| Front (`camera_link`) | Front elevated | Forward | House detection |
| Bottom-Middle (`camera_linkBM`) | Center bottom | Rear | Line following |
| Bottom-Left (`camera_linkBL`) | Left bottom | Rear | Left intersection detection |
| Bottom-Right (`camera_linkBR`) | Right bottom | Rear | Right intersection detection |

**Camera Specifications:**
- Resolution: 640×480 pixels (R8G8B8)
- Horizontal FOV: ~80° (1.3962634 radians)
- Update Rate: 3 Hz
- Range: 0.1m to 15m 

## Software Architecture

### Simulation Integration
The robot integrates with Gazebo simulator through three main plugins:

1. **Differential Drive Plugin** (`gz-sim-diff-drive-system`)
   - Subscribes to `/cmd_vel` for velocity commands
   - Publishes odometry to `/odom`
   - Controls rear wheel joints with 100 Nm max torque 

2. **Joint State Publisher** (`gz-sim-joint-state-publisher-system`)
   - Monitors all wheel joints
   - Publishes to `/joint_states` 

3. **Camera System** (`gz-sim-camera-system`)
   - Four instances for each camera
   - Publishes raw images and camera info

### Navigation System
The robot includes autonomous navigation capabilities with TSP (Traveling Salesman Problem) optimization for delivery missions. The navigation client handles route planning and execution between target houses. 

## File Structure

```
src/
├── my_robot_description/
│   └── urdf/
│       ├── my_robot.urdf.xacro      # Main robot assembly
│       ├── mobile_base.xacro        # Physical structure
│       ├── mobile_base_gazebo.xacro # Simulation plugins
│       ├── camera.xacro             # Front camera
│       ├── cameraBM.xacro           # Bottom-middle camera
│       ├── cameraBL.xacro           # Bottom-left camera
│       └── cameraBR.xacro           # Bottom-right camera
└── my_robot_bringup/
    └── scripts/
        └── deliver_robot_tsp_client.py  # Navigation client
```

## Key Features

- **Autonomous Navigation**: Differential drive locomotion with odometry-based positioning
- **Multi-Camera Vision**: Four-camera system for comprehensive environmental perception
- **TSP Optimization**: Route optimization for efficient delivery missions
- **Gazebo Simulation**: Full physics simulation with realistic sensor models
- **ROS 2 Integration**: Modern ROS 2 architecture with distributed communication

## Usage

The robot is designed for autonomous delivery tasks, navigating between houses using its vision system for detection and line following. The TSP client optimizes delivery routes for maximum efficiency.

Wiki pages you might want to explore:
- [Robot Hardware Description (liv0ri/fabemia2)](/wiki/liv0ri/fabemia2#6)
