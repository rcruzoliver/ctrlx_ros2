cmake_minimum_required(VERSION 3.8)
project(ctrlx_roscon24)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-std=c++2a -O3 -Wfatal-errors -pedantic -Wall -Wextra)
endif()

add_definitions(-DCONFIG_DEFAULT_LOGLEVEL=1)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Threads REQUIRED)

include_directories(include)

add_executable(scara_state_publisher src/scara_state_publisher.cpp src/nodeCollection.cpp)
ament_target_dependencies(scara_state_publisher 
    rclcpp geometry_msgs std_msgs sensor_msgs tf2_ros)

add_executable(scara_state_manager src/scara_state_manager.cpp src/nodeCollection.cpp)
ament_target_dependencies(scara_state_manager
   rclcpp geometry_msgs std_msgs sensor_msgs tf2_ros)

install(TARGETS
  scara_state_publisher
  scara_state_manager
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
)

install(
    DIRECTORY urdf
    DESTINATION share/${PROJECT_NAME}
)

install(
    DIRECTORY meshes 
    DESTINATION share/${PROJECT_NAME})

install(
    DIRECTORY rviz 
    DESTINATION share/${PROJECT_NAME})

ament_package()

