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
**General licence**: SPDX-License-Identifier MIT

**Additional specific licenses for third party open source libraries**: See details in each specific example.

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

# Contents

## base-humble
It contains the information to build the **ros2-base-humble** snap. Specic details to build this snap can be found in _./base_humble/README.md_

In order to make your development process as efficient as possible it is recommend splitting the ROS 2 deployment into at least two snaps. A so-called base snap is used to encapsulate the ROS 2 runtime as well as libraries which are dependencies to the application. The base snap is installed on ctrlX OS once and provides the compiled runtime and libraries to one or more ROS 2 application snaps which hold your business logic. Following this approach you can reduce build times and resource footprint on the device by sharing the ROS 2 dependencies.

## simple-py    
_To be implemented._

## nrt-dl-py
_To be implemented._

## simple-cpp
This example is composed by two snaps: **ros2-listener-simple-cpp** and **ros2-talker-simple-cpp**. Specic details to build these snaps can be found in _./simple-cpp/README.md_

This example is based on the official tutorial: [Writing a simple publisher and subscriber (C++)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html). It simply shows a way to encapsulate the talker and listener nodes in snaps that can be installed in ctrlX OS. For further details about the example, have a look at the corresponding folder.

## nrt-dl-cpp
This example is composed by two snaps: **ros2-listener-nrt-dl-cpp** and **ros2-talker-nrt-dl-cpp**. Specic details to build these snaps can be found in _./nrt-dl-cpp/README.md_

This example is based on the previously presented **simple-cpp** example. In this example the ROS2 talker node reads the Data Layer and publish a message to the ROS2 DDS middleware with that information, this message is further read by a ROS2 listener node, which then writes the information back to the datalayer. This read/write process happens in a non-realtime fashion. For further details about the example, have a look at the corresponding folder.

## shm-dl
This example is composed by two snaps: **ros2-listener-shm-dl** and **ros2-talker-shm-dl**. Specic details to build these snaps can be found in _./shm-dl/README.md_. Likewise it uses a ROS2 talker node running outside ctrlX OS, the ROS2 package containing this node can be found in [this repository](cpp_pubsub). On top of that, this example interacts with the PLC official Bosch Rexroth, the PLC project (ROS2_rt) used here is stored in ROS2_rt.zip. The PLC project must be open, compiled and uplouded to the CORE with the official PLC Engineering tool. 

This example leverages the shared memory features to communicate with the ctrlX CORE Datalayer in order to make the communication between ROS2 and any other real time app faster. As a hint, the ROS2 nodes are naturally writen in C++. 

The example have a ROS2 talker running in a external device, which publish a ROS2 topic. The external device is connected in the same LAN as the ctrlX CORE, so the ROS2 DDS allows the information to flow between the devices. There is a ROS2 listener node running in ctrlX CORE that reads that externally published topic and writes to the DataLayer using shared memory. Then a PLC program reads that information using shared memory, do some computations inside the PLC program, and writes the information back to the data layer using shared memory. Finally, there is a ROS2 talker node that reads from the Datalayer using shared memory and publishes the information to the ROS2 middleware in form of a topic.

This communication process is really fast and can be even said pseudo-realtime, however, the write/read operations relative to ROS2 are scheduled by the _roscore_, whereas the PLC tasks are scheduled by the Automation Core Main Scheduler from ctrlX OS. This fact can lead to scheduling conflicts, such as race conditions, when deploying applications based on this. 

For further details about the example, have a look at the corresponding folder.

## moveit-mtc
This example is composed by a single snap: **ros2-moveit-mtc**. Specic details to build this snap can be found in _./moveit-mtc/README.md_

This example aims to show that it is possible to deploy complex ROS2 packages in ctrlX OS. To be able to run this example the user must have SSH rights with the ctrlX CORE. The example is based on the [official Pick and Place with MoveIt Task Constructor tutorial](https://moveit.picknik.ai/humble/doc/tutorials/pick_and_place_with_moveit_task_constructor/pick_and_place_with_moveit_task_constructor.html).

The snap encapsulates the moveit2, ros2_control and MTC (moveit task constructor) libraries into a snap that can be installed in ctrlX OS. This snaps contains the basis runtime from moveit2, as well as tool to emulate robotic hardware. The example presents a planning for a pick and place task. The robot can be visualized in RViz2 if a desktop Ubuntu is connected to the same LAN, and therefore ROS2 information can be exchanges thorugh the DDS middleware.

For further details about the example, have a look at the corresponding folder.

# Related How-To's in ctrlX Community
[ROS2 demo example - MoveIt2! in ctrlX OS](https://developer.community.boschrexroth.com/t5/Store-and-How-to/ROS2-demo-example-MoveIt2-in-ctrlX-OS/ba-p/89562)

[Shared memoty communication between ROS2 and PLC via the ctrlX Datalayer](https://developer.community.boschrexroth.com/t5/Store-and-How-to/SDK-Shared-memory-communication-between-ROS2-Datalayer-and-PLC/ba-p/98107)

