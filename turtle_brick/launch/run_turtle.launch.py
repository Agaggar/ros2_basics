from ament_index_python.packages import get_package_share_path
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution

from launch_ros.actions import Node

def generate_launch_description():
    turtle_brick_pkg_path = get_package_share_path('turtle_brick')
    robot_configs = turtle_brick_pkg_path / 'turtle.yaml'
    
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('turtle_brick'),
                    'show_turtle.launch.py'
                ])
            ]),
        ),
        Node(
            package='turtlesim',
            namespace='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtle_brick',
            executable='turtle_robot',
            name='turtle_robot_node',
            output='screen',
            parameters=[robot_configs]
        )
    ])