import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

from launch_ros.actions import Node


def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled

    package_name='robot' 
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )])
                # , launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
    )

    # joystick = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource([os.path.join(
    #                 get_package_share_directory(package_name),'launch','joystick.launch.py'
    #             )])
    # )


    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
        )

    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])

    controller_params_file = os.path.join(get_package_share_directory(package_name),'config','my_controllers.yaml')

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description},
                    controller_params_file]
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    # <<< manda 'r' por serial para resetear encoders >>>
    encoder_reset = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'stty -F /dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_10:20:BA:4D:92:80-if00 57600 raw -echo && printf "r\r" > /dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_10:20:BA:4D:92:80-if00'
        ],
        output='screen',
        name='encoder_reset',
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )

    # Code for delaying a node (I haven't tested how effective it is)
    # 
    # First add the below lines to imports
    # from launch.actions import RegisterEventHandler
    # from launch.event_handlers import OnProcessExit
    #
    # Then add the following below the current diff_drive_spawner
    # delayed_diff_drive_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=spawn_entity,
    #         on_exit=[diff_drive_spawner],
    #     )
    # )
    #
    # Replace the diff_drive_spawner in the final return with delayed_diff_drive_spawner

    lidar_port = '/dev/serial/by-id/usb-Arduino_LLC_Arduino_Leonardo-if00'

    neato_lidar = Node(
        package='xv_11_laser_driver',
        executable='neato_laser_publisher',
        name='neato_laser',
        output='screen',
        parameters=[
            {'port': lidar_port},
            {'baud_rate': 115200},
            {'frame_id': 'laser_frame'}
        ]
    )

    motor_on = ExecuteProcess(
        cmd=['bash', '-c', f'printf "MotorOn\\n" > {lidar_port}'],
        output='screen',
        name='lidar_motor_on',
    )

    motor_on_after_lidar = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=neato_lidar,
            on_start=[TimerAction(period=1.0, actions=[motor_on])],
        )
    )

    # --- Apagado del motor al cerrar: pendiente. ros2 launch no garantiza ejecutar
    #     OnShutdown a tiempo en Ctrl+C, así que por ahora el motor queda girando
    #     al cerrar el launch.
    
    # from launch.event_handlers import OnShutdown
    # motor_off = ExecuteProcess(
    #     cmd=['bash', '-c',
    #          f'trap "" INT TERM; for i in 1 2 3 4 5; do printf "MotorOff\\n" > {lidar_port}; sleep 0.2; done'],
    #     output='screen',
    #     name='lidar_motor_off',
    # )
    # motor_off_on_shutdown = RegisterEventHandler(
    #     event_handler=OnShutdown(on_shutdown=[motor_off])
    # )

    mpu9250_driver = Node(
        package='mpu9250driver',
        executable='mpu9250driver', 
        name='mpu9250driver_node',
        output='screen',
        parameters=[
            os.path.join(get_package_share_directory('mpu9250driver'), 'params', 'mpu9250.yaml'),
            {'frame_id': 'imu_frame'}
        ]
    )

    madgwick = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick',
        output='screen',
        parameters=[{
            'use_mag': False,       # magnetómetro descartado: heading angular lo da el giróscopo vía EKF
            'publish_tf': False,
            'world_frame': 'enu',   # East-North-Up frame, for ground robots
        }],
        # remappings=[('/imu/data_raw', '/imu')]
    )

    delayed_madgwick = TimerAction(
        period=2.0,
        actions=[madgwick]
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(get_package_share_directory(package_name), 'config', 'ekf.yaml'),
            {'use_sim_time': False}
        ]
    )

    delayed_ekf = TimerAction(
        period=3.0,
        actions=[ekf_node]
    )

    # Launch them all!
    return LaunchDescription([
        encoder_reset,
        rsp,
        # joystick,
        twist_mux,
        neato_lidar,
        motor_on_after_lidar,
        # motor_off_on_shutdown,
        delayed_controller_manager,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner,
        mpu9250_driver,
        delayed_madgwick,
        delayed_ekf,
    ])