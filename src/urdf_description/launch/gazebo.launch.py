#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    # 1) Launch Gazebo (server + client)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch', 'gazebo.launch.py'
            ])
        )
    )

    # 2) Process Xacro and prepare robot_description
    robot_description = Command([
        FindExecutable(name='xacro'), ' ',  # invoke xacro
        PathJoinSubstitution([
            FindPackageShare('urdf_description'),
            'urdf', 'go_bdx.urdf.xacro'
        ])
    ])

    # 3) Publish robot_description
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    # 4) Joint state publisher GUI (optional)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )

    # 5) Spawn robot into Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-topic',  'robot_description',
            '-entity', 'bdx',
            '-x',      '0',
            '-y',      '0',
            '-z',      '0.0'
        ],
    )

    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher,
        joint_state_publisher_gui,
        spawn_entity
    ])
