#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    # Paths to package resources
    pkg_urdf_description = FindPackageShare('urdf_description')
    pkg_gazebo_ros = FindPackageShare('gazebo_ros')

    # Launch Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_gazebo_ros, 'launch', 'gazebo.launch.py'])
        )
    )

    # Robot description
    robot_description = Command([
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([pkg_urdf_description, 'urdf', 'go_bdx.urdf.xacro'])
    ])

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    # Controller manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            PathJoinSubstitution([pkg_urdf_description, 'config', 'controllers.yaml'])
        ],
        output='screen'
    )

    # Spawn robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'bdx',
            '-x', '0',
            '-y', '0',
            '-z', '0.082'
        ],
    )

    # Controller loading
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_trajectory_controller'],
        output='screen'
    )

    # Event handler for controller loading
    delayed_controller_loads = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[
                TimerAction(
                    period=1.0,
                    actions=[load_joint_state_broadcaster, load_joint_trajectory_controller]
                )
            ]
        )
    )

    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher,
        controller_manager,
        spawn_entity,
        delayed_controller_loads
    ])