Multi-Map Navigation with Wormholes

This ROS 2 project demonstrates a robust system for a mobile robot to navigate seamlessly between multiple,
separately mapped environments. It uses a "wormhole" concept, where predefined connection points stored in an
SQLite database allow the robot to transition from one map's coordinate frame to another.

The entire system is managed by a central C++ action server that decouples the navigation stack from the 
simulation, giving it full control to stop, start, and reconfigure the Nav2 stack for dynamic map switching.
