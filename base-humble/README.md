# ROS 2 Humble on ctrlX OS
A Snap Implementation of the ROS 2 Humble Distribution building ROS 2 from source.

## Introduction

We recommend for ROS 2 applications running on a ctrlX OS this approach:

* Build and install a __base snap__ containing the ROS 2 runtime and the components used for the ctrlX Data Layer access.
* This base snap provides its components using the snap [content interface](https://snapcraft.io/docs/content-interface).
* Pack your ROS 2 application into one or more overlay snaps.
* Let these overlay snap use the ROS 2 runtime and the ctrlX Data Layer via the content interface of the __base snap__.

This documentation describes how the base snap `ros2-base` can be build.

!!! important
    The build for arm64 devices uses Docker and is in an unreleased state!

## Content of the Base Snap

The file `snap/snapcraft.yaml` defines how the base snap will be build.

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

The base snap makes its files available via the content interface `executables`.

## About

SPDX-FileCopyrightText: Copyright (c) 2023 Bosch Rexroth AG

<https://www.boschrexroth.com/en/dc/imprint/>

## Licenses

SPDX-License-Identifier: MIT
