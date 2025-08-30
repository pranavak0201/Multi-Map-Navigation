# Multi-Map Navigation with Wormholes # 

## Overview

This ROS 2 project demonstrates a robust system for a mobile robot to navigate seamlessly between multiple,
separately mapped environments. It uses a "wormhole" concept, where predefined connection points stored in an
SQLite database allow the robot to transition from one map's coordinate frame to another.

The entire system is managed by a central C++ action server that decouples the navigation stack from the 
simulation, giving it full control to stop, start, and reconfigure the Nav2 stack for dynamic map switching.

---
## Package Structure
```bash
~/multi_map_ws/src/multi_map_navigation$
├── action
│   └── NavigateTo.action
├── CMakeLists.txt
├── config
│   └── nav2_params.yaml
├── db
│   └── wormholes.db
├── include
│   └── multi_map_navigation
│       └── navigator.h
├── launch
│   ├── amcl_map1.launch.py
│   ├── cartographer_world.launch.py
│   └── main_navigator.launch.py
├── maps
│   ├── map2.pgm
│   ├── map2.yaml
│   ├── map3.pgm
│   ├── map3.yaml
│   ├── map4.pgm
│   ├── map4.yaml
│   ├── map5.pgm
│   └── map5.yaml
├── models
│   ├── map1
│   │   ├── model.config
│   │   └── model.sdf
│   └── map2
│       ├── model.config
│       └── model.sdf
├── package.xml
├── scripts
│   └── switch_map.sh
├── src
│   ├── multi_map_navigator_node.cpp
│   └── navigator.cpp
├── worlds
│   ├── map1.world
│   ├── map2.world
│   └── map3.world
└── wormholes.db

12 directories, 30 files
```
# Features
Standard navigation within a single map using the Nav2 stack.

Multi-map navigation across pre-mapped, physically connected rooms.

"Wormhole" connection points stored in a persistent SQLite database.

A central C++ action server (/navigate_to_map) that manages all high-level navigation logic.

Dynamic map switching by systematically killing and relaunching the entire Nav2 stack.

Forceful re-localization after a map switch by publishing directly to the /initialpose topic, ensuring robustness.

# Project Setup and Workflow
The environment and connections for this project were established through the following manual process:
### 1. World & Map Creation
A single Gazebo world (map3.world) was created containing two distinct rooms connected by a doorway. Each room was then mapped independently using slam_toolbox to generate two separate map files: map3.yaml and map4.yaml. This ensures each map has its own independent coordinate frame and origin point.
### 2. Wormhole Definition
To create a link between the two maps, a physical transition point in the doorway was chosen. The following process was used to find the corresponding coordinates:
1. The full Nav2 stack was launched with map3.yaml
2. The robot was driven into the center of the doorway in the Gazebo simulation.
3. RViz's "2D Pose Estimate" tool was used to fine-tune the robot's position, ensuring the laser scan perfectly aligned with the walls of map3. The resulting pose was recorded.
4. Without moving the robot in Gazebo, the Nav2 stack was stopped and relaunched with map4.yaml
5. The "2D Pose Estimate" tool was used again to re-localize the robot at the same physical doorway, but this time on map4. This new pose was recorded.

## 3. Database Integration
The two poses recorded in the previous step, which represent the same physical location in two different coordinate systems, were inserted into the wormholes.db SQLite database. An entry was created for both directions (map3 -> map4 and map4 -> map3)

## Final Wormhole Coordinates
The following coordinates were recorded and are stored in the database. They represent the same physical location (near the doorway) in their respective map frames.

From map3:

    position: (x: 3.6789, y: -2.5341, z: 0.0)

    orientation: (z: -0.2307, w: 0.9730)

From map4:

    position: (x: 3.9810, y: -2.1935, z: 0.0)

    orientation: (z: -0.3563, w: 0.9343)
## System Architecture & Code Logic
The final architecture is designed for stability by decoupling the processes.
1.The main launch file (main_navigator.launch.py) starts only two components: the Gazebo simulation and the C++ MultiMapNavigator node.
2.The C++ node's constructor is then responsible for launching the initial Nav2 and RViz stacks for the starting map (map3). This isolates the navigation stack from Gazebo, preventing it from being killed during a map switch.

When the MultiMapNavigator's action server receives a goal, it follows this logic:
1. Same-Map Goal: If the goal is on the current map, it is forwarded directly to the running Nav2 stack.
2. Multi-Map Goal:
   1. The node queries the SQLite database to find the wormhole link
   2. It commands Nav2 to navigate the robot to the wormhole's entrance pose.
   3. Upon arrival, it uses pkill to forcefully and reliably shut down all Nav2 and RViz processes
   4. It constructs and executes a system() call to directly launch a fresh nav2_bringup stack with the new map (map4).
   5. After a brief pause for the new nodes to start, it forcefully re-localizes the robot by creating a publisher and sending a single message to the /initialpose topic with the wormhole's exit coordinates. This is the most crucial step for ensuring robust localization
   6. It then relaunches RViz.
   7. Finally, it waits for the new Nav2 action server to become available and sends the final navigation goal
 
  ## How to Run
  Follow these steps to run the simulation and test the navigation.
   1.This command starts Gazebo and the main navigator node. The navigator node will then automatically launch the Nav2 stack and RViz with map3
``` Bash
# In your first terminal
source ~/multi_map_ws/install/setup.bash
ros2 launch multi_map_navigation main_navigator.launch.py
```
2. Send a Multi-Map Navigation Goal:
   Once everything is running and the robot is localized in map3, open a second terminal. This command will instruct the robot to navigate to a point in map4, triggering the full wormhole logic.
``` Bash
# In your second terminal
source ~/multi_map_ws/install/setup.bash
ros2 action send_goal /navigate_to_map multi_map_navigation/action/NavigateTo "{
  target_map_name: 'map4', 
  target_pose: {
    header: {frame_id: 'map'}, 
    pose: {
      position: {x: -2.0, y: -2.0, z: 0.0}, 
      orientation: {w: 1.0}
    }
  }
}"
```
## VIDEO
https://drive.google.com/file/d/122HBQ53ug4vBlVJlVGUcGDoVjHFMBLfn/view?usp=sharing

   



