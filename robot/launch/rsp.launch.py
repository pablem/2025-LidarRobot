import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # Args: en real ambos quedan en false (defaults); en sim se pasan true.
    use_sim_time = LaunchConfiguration('use_sim_time')
    sim_mode = LaunchConfiguration('sim_mode')

    # Process the URDF file (con sim_mode para intercambiar el plugin de hardware)
    pkg_path = os.path.join(get_package_share_directory('robot'))
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')
    robot_description_config = ParameterValue(
        Command(['xacro ', xacro_file, ' sim_mode:=', sim_mode]),
        value_type=str
    )

    # Create a robot_state_publisher node
    params = {
        'robot_description': robot_description_config,
        'publish_frequency': 50.0,
        'tf_buffer_duration': 10.0,
        'use_sim_time': use_sim_time,
    }

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'sim_mode',
            default_value='false',
            description='Use gazebo_ros2_control hardware plugin if true'),
        node_robot_state_publisher
    ])
