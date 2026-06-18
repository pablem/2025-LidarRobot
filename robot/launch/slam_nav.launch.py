import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    package_name = 'robot'
    pkg_share = get_package_share_directory(package_name)

    # en simulación (ros2 launch ... use_sim_time:=true)
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ── SLAM Toolbox (online async, with .yaml params) ────────────────
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('slam_toolbox'),
            'launch', 'online_async_launch.py'
        )]),
        launch_arguments={
            'slam_params_file': os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml'),
            'use_sim_time': use_sim_time,
        }.items()
    )

    # ── Nav2 ─────────────────────────────────
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('nav2_bringup'),
            'launch', 'navigation_launch.py'
        )]),
        launch_arguments={
            'params_file': os.path.join(pkg_share, 'config', 'navegation2_params_waffle_mod.yaml'),
            'use_sim_time': use_sim_time,
            # 'use_respawn': 'true',   # relanza nodos caídos de Nav2
        }.items()
    )

    delayed_nav2 = TimerAction(
        period=5.0,
        actions=[nav2]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        slam,
        delayed_nav2,
    ])
