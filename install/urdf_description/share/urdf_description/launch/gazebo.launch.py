import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1) Encuentra tu paquete y la ruta al URDF
    pkg_share = get_package_share_directory('urdf_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'go_bdx.urdf')

    # 2) Lanza Gazebo (server + client)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch', 'gazebo.launch.py'
            )
        )
    )

    # 3) Publica el URDF como parámetro robot_description
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': open(urdf_file).read()
        }],
    )

    # 4) Spawnea el modelo usando -file (ruta absoluta al URDF)
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-file',    urdf_file,   # <— aquí le pasamos el URDF
            '-entity',  'bdx',       # nombre en Gazebo
            '-x',       '0',         # opcional: posición inicial X
            '-y',       '0',         #                    Y
            '-z',       '0.1'        #                    Z (un poco elevado)
        ],
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
    ])
