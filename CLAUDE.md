# CLAUDE.md

Guía para Claude Code en este repositorio. Todo el stack fue probado y funciona en el robot real. Falta implementar y probar exploración con aplicaciones que consuman nav2.  

## Build

```bash
# Desde el workspace root (~/robotLidar), no desde src/
colcon build --symlink-install
source install/setup.bash

# Build de un solo paquete
colcon build --symlink-install --packages-select diffdrive_arduino
```

Dependencias del sistema: `libserial-dev`, `ros-humble-xacro`, `ros-humble-ros2-control`, `ros-humble-ros2-controllers`, `ros-humble-slam-toolbox`, `ros-humble-nav2-bringup`

## Correr el robot

```bash
# Stack completo de hardware (motores + LiDAR + ros2_control + twist_mux)
ros2 launch robot launch_robot.launch.py

# SLAM + Nav2 + retorno autónomo a base (ejecutar después del anterior)
ros2 launch robot slam_nav.launch.py
```

## Arquitectura

Cuatro paquetes:

- **`robot/`** — Launch files, URDF/xacro, configs. Sin código ejecutable.
- **`diffdrive_arduino/`** — Plugin `ros2_control` (`DiffDriveArduinoHardware`) que se comunica con un ESP32 por serial. Lee encoders (`e\r`) y envía comandos de motor (`m val1 val2\r`).
- **`xv_11_laser_driver/`** — Nodo ROS 2 (`neato_laser_publisher`) que lee un LiDAR Neato XV-11 via serial (Arduino Leonardo) y publica `sensor_msgs/LaserScan` en `/scan`.
- **`serial/`** — Librería C++ de serial de terceros, usada por `diffdrive_arduino`.

### Flujo de datos

```
LiDAR Neato XV-11
  → (serial/USB) → xv_11_laser_driver → /scan

Encoders/Motores (ESP32)
  → (serial/USB) → diffdrive_arduino (ros2_control)
  → controller_manager
      ├── diff_drive_controller → /odom, /tf (odom→base_link)
      └── joint_state_broadcaster → /joint_states

/cmd_vel (teleop, Nav2) → twist_mux → diff_drive_controller

/scan + /odom + /tf → slam_toolbox → /map
/map + Nav2 → goals de navegación autónoma
return_to_base.py → envía goal (0,0,0) tras timeout → guarda mapa
```

## Exploración autónoma (próximamente)

Herramientas y algoritmos a definir. Esta sección se completará antes de comenzar la implementación.

## Tests

Sin tests implementados. El scaffolding existe (`ament_cmake_gtest` en `package.xml`) pero sin archivos de test.
