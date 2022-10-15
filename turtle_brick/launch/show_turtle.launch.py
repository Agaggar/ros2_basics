from ament_index_python.packages import get_package_share_path
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    turtle_brick_pkg_path = get_package_share_path('turtle_brick')
    default_model_path = turtle_brick_pkg_path / 'turtle.urdf.xacro'
    default_rviz_config_path = turtle_brick_pkg_path / 'turtle_urdf.rviz'

    gui_arg = DeclareLaunchArgument(name='gui', default_value='true', choices=['true', 'false'],
                                    description='Flag to enable joint_state_publisher_gui')
    model_arg = DeclareLaunchArgument(name='model', default_value=str(default_model_path),
                                      description='Absolute path to robot urdf file')
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
                                     description='Absolute path to rviz config file')

    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)
    
    wheel_radius = LaunchConfiguration('wheel_radius')
    platform_height = LaunchConfiguration('platform_height')
    max_velocity = LaunchConfiguration('max_velocity')
    gravity = LaunchConfiguration('gravity')
    robot_configs = turtle_brick_pkg_path / 'turtle.yaml'
    # assert that platform_height is >=7*wheel_radius

    # gravity_arg = DeclareLaunchArgument('gravity', default_value='9.8')

    arena_node = Node(
        package='turtle_brick',
        executable='arena',
        output='screen',
        emulate_tty=True,
        # parameters=[robot_configs]
    )
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name="robot_publisher",
        parameters=[{'robot_description': robot_description}]
    )

    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    # parameter should be called use_jsp: gui, jsp, or none
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        # output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    '''
    <node pkg="turtlesim" exec="turtlesim_node" name="roving_turtle">
        <param from="$(find-pkg-share turtle_control)/colors.yaml" />
   </node>
    '''

    return LaunchDescription([
        gui_arg,
        model_arg,
        rviz_arg,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        arena_node,
        rviz_node
    ])