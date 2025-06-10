# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Example usages:
    # - ros2 launch orcahand_description orcahand.launch.py
    # - ros2 launch orcahand_description orcahand.launch.py urdf_file:=orcahand_left.urdf

    # Launch argument for URDF filename
    urdf_file_arg = DeclareLaunchArgument(
        'urdf_file',
        default_value='orcahand_right.urdf',
        description='URDF file to load from the orcahand_description/urdf directory'
    )

    # Construct the full path to the URDF/XACRO file using the argument
    orcahand_description_path = PathJoinSubstitution([
        FindPackageShare('orcahand_description'),
        'urdf',
        LaunchConfiguration('urdf_file')
    ])

    # RViz config path (static)
    orcahand_rviz_config_path = PathJoinSubstitution([
        FindPackageShare('orcahand_description'),
        'rviz',
        'config.rviz'
    ])

    return LaunchDescription([
        urdf_file_arg,

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command([
                    'xacro ', orcahand_description_path
                ])
            }],
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', orcahand_rviz_config_path],
        ),
    ])
