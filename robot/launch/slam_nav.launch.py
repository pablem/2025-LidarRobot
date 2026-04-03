import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'robot'
    pkg_share = get_package_share_directory(package_name)

    # ── SLAM Toolbox (online async, with .yaml params) ────────────────
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('slam_toolbox'),
            'launch', 'online_async_launch.py'
        )]),
        launch_arguments={
            'slam_params_file': os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml'),
            'use_sim_time': 'false',
        }.items()
    )

    # ── Nav2 ─────────────────────────────────
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('nav2_bringup'),
            'launch', 'navigation_launch.py'
        )]),
        launch_arguments={
            'use_sim_time': 'false',
        }.items()
    )

    delayed_nav2 = TimerAction(
        period=5.0,
        actions=[nav2]
    )

    # ── Nodo retorno a base ───────────────────────────────────────────────
    return_to_base = Node(
        package='robot',
        executable='return_to_base',          # (registrado en CMakeLists.txt)
        name='return_to_base',
        output='screen',
        parameters=[{
            'exploration_time': 150.0,        # segundos
            'map_dir': '/home/pablo',
            'map_base_name': 'labo',
            'overwrite_map': False,           # True = labo_temp
        }]
    )

    delayed_return_to_base = TimerAction(
        period=10.0,
        actions=[return_to_base]
    )

    return LaunchDescription([
        slam,
        delayed_nav2,
        delayed_return_to_base,
    ])