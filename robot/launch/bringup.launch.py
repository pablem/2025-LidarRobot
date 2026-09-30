"""
bringup.launch.py
launch_robot + slam_nav en una sola terminal.

la espera entre ambos — slam_toolbox y Nav2 arrancan recién cuando el hardware
está publicando /scan y /odom. La espera consulta el grafo ROS
(scripts/wait_for_hardware.py), no un timer fijo.

    ros2 launch robot bringup.launch.py
    ros2 launch robot bringup.launch.py hardware_timeout:=90.0
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'robot'
    pkg_share = get_package_share_directory(package_name)

    use_sim_time = LaunchConfiguration('use_sim_time')
    hardware_timeout = LaunchConfiguration('hardware_timeout')

    # ── launches hardware y slam juntos  ────────────────────
    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            pkg_share, 'launch', 'launch_robot.launch.py'
        )])
    )

    slam_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            pkg_share, 'launch', 'slam_nav.launch.py'
        )]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # ── Espera al hardware ────────────────────────────────────────────
    # Corre en paralelo a `hardware` y termina apenas el grafo ROS muestra
    # los nodos y publishers de launch_robot.
    wait_hardware = Node(
        package=package_name,
        executable='wait_for_hardware',
        name='wait_for_hardware',
        output='screen',
        arguments=['--timeout', hardware_timeout],
    )

    def _on_hardware_ready(event, context):
        if context.is_shutdown:
            # Ctrl+C durante la espera: no arrancar nada más
            return None
        if event.returncode == 0:
            return [LogInfo(msg='[bringup] hardware listo, arranca SLAM + Nav2'),
                    slam_nav]
        # Se agotó el tiempo pero el hardware puede estar bien igual: el
        # chequeo por grafo ROS a veces no confirma por motivos ajenos al
        # proceso real (mismo criterio que el menú). Se sigue, con el aviso
        # a la vista, en vez de dejar al usuario sin SLAM ni Nav2.
        return [LogInfo(msg='[bringup] el hardware no confirmó a tiempo; '
                            'arranca SLAM + Nav2 igual (revisá /scan y /odom)'),
                slam_nav]

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Reloj de simulación (se pasa a slam_nav)'),
        DeclareLaunchArgument(
            'hardware_timeout',
            default_value='60.0',
            description='Segundos a esperar a que el hardware publique /scan y /odom'),
        hardware,
        wait_hardware,
        RegisterEventHandler(
            OnProcessExit(target_action=wait_hardware, on_exit=_on_hardware_ready)),
    ])
