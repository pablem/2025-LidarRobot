import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    """Simulación en Gazebo Fortress (Ignition Gazebo 6) + gz_ros2_control.

    El robot reutiliza el MISMO ros2_control que el real (diff_cont + joint_broad,
    via my_controllers.yaml). LiDAR e IMU se publican en gz transport y se traen
    a ROS con ros_gz_bridge. El EKF y twist_mux corren nativos en ROS.
    """

    package_name = 'robot'
    pkg_share = get_package_share_directory(package_name)

    # robot_state_publisher con tiempo de simulación y plugins de Gazebo (sim_mode)
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            pkg_share, 'launch', 'rsp.launch.py'
        )]),
        launch_arguments={'use_sim_time': 'true', 'sim_mode': 'true'}.items()
    )

    # Gazebo Fortress con el mundo de obstáculos (-r: corre al iniciar)
    world = os.path.join(pkg_share, 'worlds', 'obstacles_gz.sdf')
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': ['-r -v4 ', world]}.items()
    )

    # Spawnea el robot desde /robot_description
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'robot', '-z', '0.1'],
        output='screen'
    )

    # Puente gz <-> ROS: reloj de simulación + sensores
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/imu/data_raw@sensor_msgs/msg/Imu[ignition.msgs.IMU',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # twist_mux (misma config que el real), salida al diff_drive_controller
    twist_mux_params = os.path.join(pkg_share, 'config', 'twist_mux.yaml')
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[twist_mux_params, {'use_sim_time': True}],
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')]
    )

    # Spawners de controllers (el controller_manager lo crea el plugin
    # gz_ros2_control una vez que el robot está spawneado en Gazebo)
    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont'],
    )

    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad'],
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[diff_drive_spawner],
        )
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_broad_spawner],
        )
    )

    # EKF (mismo ekf.yaml que el real), publica odom->base_link y /odom
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(pkg_share, 'config', 'ekf.yaml'),
            {'use_sim_time': True}
        ],
        remappings=[('odometry/filtered', '/odom')]
    )

    return LaunchDescription([
        rsp,
        gz_sim,
        spawn_entity,
        bridge,
        twist_mux,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner,
        ekf_node,
    ])
