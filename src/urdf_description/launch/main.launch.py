from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess
from launch.substitutions import FindExecutable


def generate_launch_description():
    pkg_urdf = FindPackageShare('urdf_description')
    pkg_gazebo = FindPackageShare('gazebo_ros')

    urdf_file = PathJoinSubstitution([
        pkg_urdf,
        'urdf',
        'go_bdx.urdf.xacro'
    ])

    robot_description = Command([
        FindExecutable(name='xacro'), ' ', urdf_file
    ])

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            pkg_gazebo, '/launch/gazebo.launch.py'
        ])
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # ros2_control node
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            PathJoinSubstitution([pkg_urdf, 'config', 'controllers.yaml'])
        ],
        output='screen'
    )

    # Spawn entity in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'bdx',
            '-x', '0', '-y', '0', '-z', '0.082'
        ],
        output='screen'
    )

    # Load controllers
    load_jsb = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    load_jtc = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_trajectory_controller'],
        output='screen'
    )

    delayed_load = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[
                TimerAction(
                    period=1.0,
                    actions=[load_jsb, load_jtc]
                )
            ]
        )
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        control_node,
        spawn_entity,
        delayed_load,
        rviz_node
    ])
