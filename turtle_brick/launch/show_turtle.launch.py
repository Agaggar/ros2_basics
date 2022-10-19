from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    turtle_brick_pkg_path = get_package_share_path('turtle_brick')
    default_model_path = turtle_brick_pkg_path / 'turtle.urdf.xacro'
    default_rviz_config_path = turtle_brick_pkg_path / 'turtle_urdf.rviz'

    model_arg = DeclareLaunchArgument(name='model', default_value=str(default_model_path),
                                      description='Absolute path to robot urdf file')
    rviz_arg = DeclareLaunchArgument(name='rvizconfig',
                                     default_value=str(default_rviz_config_path),
                                     description='Absolute path to rviz config file')
    use_jsp = DeclareLaunchArgument(name='use_jsp', default_value='none',
                                    choices=['gui', 'jsp', 'none'],
                                    description='Choices for joint state publisher gui')

    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)
    robot_configs = turtle_brick_pkg_path / 'turtle.yaml'

    arena_node = Node(
        package='turtle_brick',
        executable='arena',
        output='screen',
        emulate_tty=True,
        parameters=[robot_configs]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        # name="robot_publisher",
        parameters=[{'robot_description': robot_description}]
    )

    # Depending on gui parameter, either launch joint_state_publisher, joint_state_publisher_gui,
    # or none
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=LaunchConfigurationEquals('use_jsp', 'jsp')
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=LaunchConfigurationEquals('use_jsp', 'gui')
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        # output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    return LaunchDescription([
        model_arg,
        rviz_arg,
        use_jsp,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        arena_node,
        rviz_node
    ])
