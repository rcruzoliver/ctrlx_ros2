# Rviz desktop application for SCARA robot  

This folder includes a ROS2 workspace with the source code to compile a Rviz application that visualizes a SCARA robot.

It subscribes to the topic /scara_joints published by the snap scara-publisher-cpp.


## Build
To build the workspace call:

```bash
./colcon-build.sh
```

## Run
To launch the application call:

```bash
./run.sh
```