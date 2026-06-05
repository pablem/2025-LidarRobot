import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory('robot')

    # ── Undock: avanza x cm en línea recta para salir de la terminal de carga ──
    undock = Node(
        package='robot',
        executable='undock',
        name='undock',
        output='screen',
        parameters=[{
            'undock_dist': 0.40,   # metros
            'undock_speed': 0.10,  # m/s
        }]
    )

    # ── explore_lite: exploración autónoma de fronteras ──────────────────
    explore = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('explore_lite'),
            'launch', 'explore.launch.py'
        )]),
        launch_arguments={
            'use_sim_time': 'false',
        }.items()
    )

    # Esperar a que undock termine antes de explorar (x seg de maniobra + margen)
    delayed_explore = TimerAction(period=5.0, actions=[explore])

    # ── return_to_base: monitorea exploración, vuelve al dock y guarda mapa ─
    return_to_base = Node(
        package='robot',
        executable='return_to_base',
        name='return_to_base',
        output='screen',
        parameters=[{
            'exploration_time': 540.0, #540.0,  
            'idle_timeout': 60.0,  
            'map_dir': '/home/pablo',
            'map_base_name': 'explore',
            'overwrite_map': True,
            # Maniobra de docking: navega a (dock_x_offset, 0) y retrocede al dock
            'dock_x_offset': 0.40,      # metros delante del dock
            'dock_reverse_dist': 0.56,  # metros de retroceso final
            'dock_speed': 0.10,         # m/s
            'battery_topic': '', #'''/battery_state',
            'battery_threshold': 10.85,  # voltaje (V) del pack
        }]
    )

    # Esperar a que Nav2 y explore_lite estén listos
    delayed_return_to_base = TimerAction(period=12.0, actions=[return_to_base])

    return LaunchDescription([
        undock,
        delayed_explore,
        delayed_return_to_base,
    ])
