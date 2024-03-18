# ctrlx_ros2

This is a non-official repository maintained by Raul Cruz-Oliver (raul.cruz.oliver@gmail.com) that contains an ehanced SDK for solutions that aim to integrate ROS2 into ctrlX OS. It is based on the [official ROS2-SDK](https://github.com/boschrexroth/ctrlx-automation-sdk-ros2) provided by Bosch Rexroth 

Use this repository at your own risk and please contact the maintainer if you detect any bugs.

$~$

# Important directions for use
### Areas of use and application
The content (e.g. source code and related documents) of this repository is intended to be used for configuration, parameterization, programming or diagnostics in combination with selected Bosch Rexroth ctrlX AUTOMATION devices. Additionally, the specifications given in the "Areas of Use and Application" for ctrlX AUTOMATION devices used with the content of this repository do also apply.

### Unintended use
Any use of the source code and related documents of this repository in applications other than those specified above or under operating conditions other than those described in the documentation and the technical specifications is considered as "unintended". Furthermore, this software must not be used in any application areas not expressly approved by Bosch Rexroth

### Licenses
SPDX-License-Identifier: MIT

$~$

# Device requirements

## Running platform
ROS2 is designed for Ubuntu 22.04 LTS (and Windows and macOS), thus we need to run ctrlX OS 2.xx, i.e. the one based on a ubuntu core 22.

For this SDK we have chosen ROS2 Humble Hakswill, as it is the newest LTS distribution. 

## Compilation platform
Snapcraft is a technology from Canonical that only runs on Ubuntu. In particular, these examples are designed to be compiled in Ubuntu 22.04 LTS, no matter if it a native installation or a virtual machine. If you want to know more about snapcraft please visit the [official documentation](https://snapcraft.io/docs/snapcraft).

There are two types of processing architecture available in the ctrlX CORE devices: arm64 in the case of ctrlX X3 and amd64 in the case of ctrlX virtual, X5 and X7. ROS2 is supported in both of them. However, so far, it is only possible to compile the ROS2 related snaps in the native architecture in which they will run, i.e. cross-compilation is not supported. 

As a hint, Intel or AMD processors have amd64 architecture, whereas we find arm64 architectures in the Apple Silicon chips, or in platform such as Raspberry Pi. The examples included in this repository have been compiled with an Intel i7-1270P installed in a Lenovo ThinkPad 16T running Ubuntu in a virtual machine when targeting amd64, and with a Broadcom BCM2711 Quad core Cortex-A72 installed in a Raspberry Pi model 4 running Ubuntu natively when targeting arm64. If you manage to compile the arm64 examples using an Apple Silicon CPU, please let me know.

The snapcraft.yalm files contained in this repository can be identically used in both architectures, 

$~$

# Examples

## base-humble
This example contains the information to build the **ros2-base-humble** snap.

In order to make your development process as efficient as possible it is recommend splitting the ROS 2 deployment into at least two snaps. A so-called base snap is used to encapsulate the ROS 2 runtime as well as libraries which are dependencies to the application. The base snap is installed on ctrlX OS once and provides the compiled runtime and libraries to one or more ROS 2 application snaps which hold your business logic. Following this approach you can reduce build times and resource footprint on the device by sharing the ROS 2 dependencies.

To build the snap run:

```bash
cd <yourPath>/ctrlx_ros2/base-humble
./build-snap.sh
```
To know more about this example visit the corresponding folder.

## simple-listener|talker-cpp
This example is composed by two snaps: **ros2-simple-listener-cpp** and **ros2-simple-talker-cpp**

To build **ros2-simple-listener-cpp** run the following:
```bash
cd <yourPath>/ctrlx_ros2/simple-listener-cpp
./build-snap.sh
```

To build **ros2-simple-talker-cpp** run the following:
```bash
cd <yourPath>/ctrlx_ros2/simple-talker-cpp
./build-snap.sh
```
To know more about this example visit the corresponding folder.

## simple-listener|talker-dl-cpp
This example is composed by two snaps: **ros2-simple-listener-dl-cpp** and **ros2-simple-talker-dl-cpp**

To build **ros2-simple-listener-dl-cpp** run the following:
```bash
cd <yourPath>/ctrlx_ros2/simple-listener-cpp
./build-snap.sh
```

To build **ros2-simple-talker-dl-cpp** run the following:
```bash
cd <yourPath>/ctrlx_ros2/simple-talker-cpp
./build-snap.sh
```
To know more about this example visit the corresponding folder.

## listener|talker-rt-dl
This example is composed by two snaps: **ros2-simple-listener-dl-cpp** and **ros2-simple-talker-dl-cpp**

To build **ros2-simple-listener-dl-cpp** run the following:
```bash
cd <yourPath>/ctrlx_ros2/simple-listener-cpp
./build-snap.sh
```

To build **ros2-simple-talker-dl-cpp** run the following:
```bash
cd <yourPath>/ctrlx_ros2/simple-talker-cpp
./build-snap.sh
```

To know more about this example visit the corresponding folder.

## moveit-mtc
This example is composed by a single snap: **ros2-moveit-mtc** 

To build **ros2-moveit-mtc** run the following:
```bash
cd <yourPath>/ctrlx_ros2/moveit-mtc
./build-snap.sh
```
To know more about this example visit the corresponding folder.

# Related How-To's in ctrlX Community
[ROS2 demo example - MoveIt2! in ctrlX OS](https://developer.community.boschrexroth.com/t5/Store-and-How-to/ROS2-demo-example-MoveIt2-in-ctrlX-OS/ba-p/89562)

