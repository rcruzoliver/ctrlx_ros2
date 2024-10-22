# Author: Raul Cruz-Oliver
# Email: raul.cruz.oliver@gmail.com
# Date, place: January 2023, Butikkon, CH

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def static_transform(parent, child, position):
    return Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[position[0], position[1], position[2], '0', '0', '0', parent, child]
        )

def generate_launch_description():

    urdf_file = os.path.join(get_package_share_directory('ctrlx_roscon24'), 'urdf', 'scara.urdf')
    
    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            arguments=[urdf_file])
    
    scara_state_publisher_node = Node(
            package='ctrlx_roscon24',          
            executable='scara_state_publisher',
            name='scara_state_publisher',
            output='screen')
    
    scara_state_manager_node = Node(
            package='ctrlx_roscon24',          
            executable='scara_state_manager',
            name='scara_state_manager',
            arguments=["jog_demo"],  
            output='screen')
    
    rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", os.path.join(get_package_share_directory("ctrlx_roscon24"), "rviz", "scara.rviz")])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation/Gazebo clock if true'),    
        robot_state_publisher,
        scara_state_publisher_node,
        rviz_node
        ])
