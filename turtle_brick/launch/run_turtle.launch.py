from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution

from launch_ros.actions import Node

def generate_launch_description():
    turtle_yaml_path = PathJoinSubstitution([
                    FindPackageShare('turtle_brick'),
                    'turtle.yaml'  
                ])
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('turtle_brick'),
                    'show_turtle.launch.py'
                ])
            ]),
            launch_arguments={
                'wheel_radius': '0.5',
                'platform_height': '6.0',
                'max_velocity': '0.22',
                'gravity': '9.8'
            }.items()
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
            name='turtle_robot_node'
        )
    ])