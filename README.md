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


