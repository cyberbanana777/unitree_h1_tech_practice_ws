from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='aruco_analyzer',
            executable='aruco_analyzer',
            name='aruco_analyzer',  # должно совпадать с именем в YAML
            output='screen',
            parameters=[
                # Загружаем параметры из YAML-файла внутри пакета
                PathJoinSubstitution([
                    FindPackageShare('aruco_analyzer'),
                    'config', 'release_params.yaml'
                ]),
            ]
        )
    ])