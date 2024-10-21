# ROS 2 Humble on ctrlX OS
A Snap Implementation of the ROS 2 Humble Distribution building ROS 2 from source.

## Build
To build this snap you simply need to call:

```bash
./build-snap.sh
```

## About
### Debian Packages

The snapcraft plugin [colcon](https://snapcraft.io/docs/colcon-plugin) is used to download Debian packages:

* Needed during build process: make, gcc, g++
* Needed at runtime: software-properties-common, ros-humble-ros-base, python3-argcomplete, ca-certificates, libzmq5, ctrlx-datalayer

### Python Wheels

The snapcraft plugin [python](https://snapcraft.io/docs/python-plugin) is used to download python wheels:

* python3-wheel
* ctrlx-datalayer
* empy
* numpy
* rosdep 
* rosdistro
* colcon-core
* lark rosdep 
* rosdistro
* colcon-core
* lark

### Content Interface

The base snap makes its files available via the content interface `executables`. These files (ROS2 runtime) will be then later used by other snaps that use ROS2 functinalities.


## License
Snap belonging to ctrlx_ros2 repository, ergo protected under license MIT.
