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

# SLAM + Nav2 (ejecutar después del anterior)
ros2 launch robot slam_nav.launch.py

# Exploración autónoma con explore_lite (ejecutar después de los dos anteriores)
ros2 launch explore_lite explore.launch.py
```

## Arquitectura

Cinco paquetes:

- **`robot/`** — Launch files, URDF/xacro, configs. Sin código ejecutable.
- **`diffdrive_arduino/`** — Plugin `ros2_control` (`DiffDriveArduinoHardware`) que se comunica con un ESP32 por serial. Lee encoders (`e\r`), envía comandos de motor (`m val1 val2\r`) y lee la tensión de batería (`b\r`). Publica `sensor_msgs/msg/BatteryState` en `/battery_state` y minutos restantes estimados en `/battery_time_remaining` (ver "Sensor de batería").
- **`xv_11_laser_driver/`** — Nodo ROS 2 (`neato_laser_publisher`) que lee un LiDAR Neato XV-11 via serial (Arduino Leonardo) y publica `sensor_msgs/LaserScan` en `/scan`.
- **`m-explore-ros2/`** — Exploración autónoma de fronteras (`explore_lite`). Detecta fronteras en el costmap de Nav2 y envía goals via `NavigateToPose`. Probado y funciona.
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

Batería (ESP32, comando `b`)
  → (serial/USB) → diffdrive_arduino → /battery_state (sensor_msgs/BatteryState)
                                     → /battery_time_remaining (std_msgs/Float32, minutos)

/scan + /odom + /tf → slam_toolbox → /map
/map + Nav2 → goals de navegación autónoma
return_to_base.py → envía goal (0,0,0) tras timeout → guarda mapa

/map (costmap Nav2) → explore_lite → FrontierSearch → NavigateToPose → Nav2
```

## Exploración autónoma

Usa **`explore_lite`** del paquete `m-explore-ros2`. Probado el 2026-05-19, funciona correctamente.

### Comportamiento observado
- El costmap de Nav2 creció de 64×108 a 113×127 celdas (0.05 m/px) durante la sesión, mapeando ~5.6m × 6.4m.
- explore_lite preempta goals frecuentemente (~3s) a medida que el mapa crece y aparecen mejores fronteras — comportamiento normal.
- Dos warnings `BehaviorTree tick rate 100.00 exceeded` son esperables en Raspberry Pi bajo carga.
- Al hacer Ctrl+C el goal en curso se cancela limpiamente.

### Parámetros clave (`m-explore-ros2/explore/config/params.yaml`)
| Parámetro | Valor | Descripción |
|---|---|---|
| `planner_frequency` | 0.33 Hz | Frecuencia de re-evaluación de fronteras |
| `progress_timeout` | 30 s | Tiempo sin avance antes de blacklistear un goal |
| `min_frontier_size` | 0.75 m | Tamaño mínimo de frontera a perseguir |
| `return_to_init` | true | Vuelve al origen al terminar la exploración |
| `potential_scale` | 3.0 | Peso de la distancia al goal |
| `gain_scale` | 1.0 | Peso del tamaño de la frontera |

### Depuración
```bash
# Ver goals enviados a Nav2 en tiempo real
ros2 topic echo /navigate_to_pose/_action/status

# Ver estado de exploración
ros2 topic echo /explore/status

# Ver fronteras en RViz (requiere visualize: true en params.yaml)
# Topic: /explore/frontiers
```

## Sensor de batería

El firmware del ESP32 responde al comando `b\r` con la tensión total del pack (4S LiPo). El hardware interface la lee con `ArduinoComms::read_battery_voltage()`, le aplica una **calibración lineal contra multímetro real** (`real = 1.33333 * raw + 0.20`, ajustada con los puntos raw→real `11.4→15.40` y `12.30→16.60`) y publica desde un nodo interno (`diffdrive_battery`):

- `/battery_state` — `sensor_msgs/msg/BatteryState`. Solo se tiene la tensión total, así que `cell_voltage` queda vacío y `current`/`charge`/`capacity` van como NaN. `percentage` se calcula linealmente entre `battery_voltage_min` y `battery_voltage_max`. `power_supply_technology` = LIPO.
- `/battery_time_remaining` — `std_msgs/msg/Float32`, minutos restantes estimados = `percentage * battery_runtime_full_min`.

La lectura se hace en `read()`, throttleada cada `battery_publish_period` segundos, **sin importar si el robot está en movimiento**. (Históricamente se sensaba solo con el robot quieto para evitar que el handler `b\r` — que perdía ticks de encoder al deshabilitar interrupciones durante `analogRead()` — degradara la odometría; ese guard fue removido.)

### Parámetros (`robot/description/ros2_control.xacro`, todos opcionales con defaults en el código)
| Parámetro | Default | Descripción |
|---|---|---|
| `battery_voltage_min` | 12.0 V | Tensión (real, calibrada) a 0% de carga |
| `battery_voltage_max` | 16.8 V | Tensión (real, calibrada) a 100% de carga |
| `battery_runtime_full_min` | 120 min | Autonomía a plena carga (uso intensivo); ajustable |
| `battery_publish_period` | 60 s | Período entre lecturas/publicaciones |

### Depuración
```bash
ros2 topic echo /battery_state
ros2 topic echo /battery_time_remaining
```

### GUI de monitoreo
Pequeña ventana Tkinter (`robot/scripts/battery_monitor.py`) con voltaje, barra de carga coloreada (verde >50 %, naranja 20–50 %, rojo <20 %), porcentaje, minutos restantes y estado. RViz2 **no** tiene display nativo para `BatteryState`, por eso esta GUI. **Corre en la PC, no en la Raspberry** (que es headless); se suscribe por la red DDS.
```bash
ros2 run robot battery_monitor
```

## Tests

Sin tests implementados. El scaffolding existe (`ament_cmake_gtest` en `package.xml`) pero sin archivos de test.
