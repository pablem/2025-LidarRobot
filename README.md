# 2025-LidarRobot

**Autonomous differential-drive robot** built on a Raspberry Pi 4 running ROS 2 Humble. It uses a recycled Neato XV-11 LiDAR for 2D perception, an ESP32-S3 as the low-level motor controller, and the full ROS 2 navigation stack (ros2_control · SLAM Toolbox · Nav2).

> 📖 Full documentation (Spanish): [Robot LiDAR 2025 – Notion](https://www.notion.so/352ce5a2286e801ca74dd87a884d5030)

---

## System Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                       Raspberry Pi 4                            │
│                       Ubuntu 22.04 LTS                          │
│                       ROS 2 Humble                              │
│                                                                 │
│  /cmd_vel ──► twist_mux ──► diff_drive_controller               │
│                                    │                            │
│                          diffdrive_arduino (HW Interface)       │
│                                    │  serial 57600 baud         │
└────────────────────────────────────┼────────────────────────────┘
                                     │ USB
                              ┌──────┴──────┐
                              │  ESP32-S3   │  PI controllers
                              │  firmware   │  PCNT encoders
                              └──────┬──────┘  MCPWM motors
                                     │
                          L6201P ─── motors ─── AS5040 encoders

Sensors
  LiDAR: Neato XV-11  ──► /scan ──► SLAM Toolbox ──► Nav2
  IMU:   MPU-9250     ──► /imu/data_raw
                      ──► Madgwick filter ──► /imu/data
                      ──► robot_localization EKF ──► /odom ──► Nav2
```

**Frame tree:**

```
map → odom → base_footprint → base_link → chassis → laser_frame
                                                   → imu_frame
```

---

## Repository Structure

```
src/
├── robot/                   # Main ROS 2 package
├── diffdrive_arduino/       # ros2_control hardware interface
├── serial/                  # C++ serial library
├── lidar_driver/            # XV-11 ROS 2 driver (fork for Humble)
├── mpu9250driver/           # IMU driver
└── firmware/                # ESP32-S3 PlatformIO project
```

### `robot/` (package)

Main robot package. Contains launchers, URDF/Xacro description, and all configuration files.

| Directory | Contents |
|-----------|----------|
| `launch/` | `launch_robot.launch.py` (real robot), `launch_sim.launch.py` (Gazebo) PENDING |
| `description/` | `robot.urdf.xacro`, `ros2_control.xacro`, `lidar.xacro`, `imu.xacro` |
| `config/` | `my_controllers.yaml`, `mapper_params_online_async.yaml`, `nav2_params.yaml`, `ekf.yaml`, `twist_mux.yaml`, RViz2 configs |

Forked from: [joshnewans/my_bot](https://github.com/joshnewans/my_bot)

---

### `diffdrive_arduino/` (ros2_control hardware interface)

Implements the `hardware_interface::SystemInterface` for `ros2_control`. Communicates with the ESP32-S3 firmware over serial at 57600 baud using a simple ASCII protocol:

| Command | Action |
|---------|--------|
| `e\r` | Read encoder counts from both wheels |
| `m <spd1> <spd2>\r` | Set closed-loop velocity targets (ticks/sample) |
| `r\r` | Reset encoder counters |

The `diff_drive_controller` calls this interface every control cycle to read wheel positions/velocities and send updated speed commands.

Copied from: [joshnewans/diffdrive_arduino](https://github.com/joshnewans/diffdrive_arduino)  
Based on: [ros-controls/ros2_control_demos – example_2](https://github.com/ros-controls/ros2_control_demos/tree/master/example_2)

---

### `serial/` (library)

C++ library for synchronous/asynchronous serial communication used internally by `diffdrive_arduino`.

Copied from: [joshnewans/serial](https://github.com/joshnewans/serial)

---

### `lidar_driver/` (XV-11 ROS 2 driver)

Modified fork of the `xv_11_laser_driver` package, ported to **ROS 2 Humble**. Publishes `/scan` (`sensor_msgs/LaserScan`) at `frame_id: laser_frame`.

The Neato XV-11 communicates at **115,200 baud**, generates 90 packets/revolution (22 bytes each), delivering 360 distance readings per revolution (15 cm – 6 m range).

Original ROS 1 driver: [ros-drivers/xv_11_laser_driver](http://wiki.ros.org/xv_11_laser_driver)  
Protocol reference: [ssloy/neato-xv11-lidar](https://github.com/ssloy/neato-xv11-lidar)

---

### `mpu9250driver/` (IMU driver)

Driver for the **MPU-9250** (MPU-6500 accel/gyro + AK8963 magnetometer) over I2C. Publishes:

- `/imu/data_raw` – raw acceleration and angular velocity (SI units)
- `/imu/mag` – raw magnetometer reading (for Madgwick filter and calibration)

Modified from [hiwad-aziz/ros2_mpu9250_driver](https://github.com/hiwad-aziz/ros2_mpu9250_driver). Changes: orientation estimation removed (delegated to Madgwick filter), block reads, SI unit conversion, magnetometer bias support, covariance corrections.

---

### `firmware/` (ESP32-S3 – PlatformIO)

Low-level motor controller running on the **ESP32-S3** (PlatformIO + Arduino framework). Acts as a serial bridge between ROS 2 and the two DC motors.

Key features:
- **Closed-loop PI control** at 100 Hz (hardware timer ISR)
- **PCNT hardware counters** for quadrature encoder reading (via `ESP32Encoder`)
- **MCPWM** for dual H-bridge control at 16 kHz (Nisi L6201P drivers)
- **AS5040** magnetic encoders (incremental A/B output)
- **OTA flashing** over WiFi (`ArduinoOTA`)
- Dead-band compensation and integral anti-windup

Source: [pablem/2025-ros_ESP32S3_Motor_Controller](https://github.com/pablem/2025-ros_ESP32S3_Motor_Controller)  
📖 [Firmware documentation (Notion)](https://www.notion.so/35bce5a2286e8088b98bc4f5186d5208)

---

## Hardware

| Component | Part |
|-----------|------|
| SBC | Raspberry Pi 4 (Ubuntu Server 22.04 LTS, 64-bit) |
| LiDAR | Neato XV-11  |
| Microcontroller | ESP32-S3 |
| IMU | MPU-9250 (I2C, address 0x68) |
| Motor drivers | Nisi L6201P × 2 |
| Encoders | AS5040 magnetic (10-bit, A/B incremental output) |
| Storage | microSD 32 GB (A2-rated) |

---

## Odometry Parameters

Calibrated values used in `ros2_control.xacro` and `my_controllers.yaml`:

| Parameter | Value |
|-----------|-------|
| `enc_counts_per_rev` (left) | 33 572 |
| `enc_counts_per_rev` (right) | 32 717 |
| `wheel_separation` | 0.408 m |
| `wheel_radius` | 0.033 m |
| Serial baud rate | 57 600 |

> The two encoders are not identical and are configured separately. Wheel separation was corrected empirically by measuring heading error after a 360° in-place rotation.

---

## Dependencies

### System (Ubuntu 22.04)

```bash
sudo apt update
sudo apt-get install libserial-dev libi2c-dev libboost-all-dev
```

### ROS 2 Humble

```bash
sudo apt install \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-gazebo-ros2-control \
  ros-humble-slam-toolbox \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-robot-localization \
  ros-humble-imu-tools \
  ros-humble-twist-mux \
  ros-humble-tf2-ros \
  ros-humble-tf2-tools \
  ros-humble-rqt-tf-tree \
  ros-humble-rmw-cyclonedds-cpp
```

---

## Build & Install

### Automated setup (Ubuntu 22.04 / WSL2)

`setup_robotlidar.sh` installs ROS 2 Humble, Gazebo Fortress and the full navigation stack (SLAM Toolbox, Nav2, IMU tools, robot_localization), clones the repo into `~/robotLidar`, resolves dependencies with `rosdep`, builds the simulation packages and configures `~/.bashrc`. Intended for running the **simulation** on a fresh Ubuntu/WSL2 machine.

```bash
git clone https://github.com/pablem/2025-LidarRobot.git
bash 2025-LidarRobot/setup_robotlidar.sh
```

### Manual build

```bash
# Create workspace
mkdir robot_ws

# Clone (this repo IS the src/ folder)
git clone https://github.com/pablem/2025-LidarRobot.git
mv 2025-LidarRobot src
cd ~/robot_ws

# Build
colcon build --symlink-install

# Source the workspace
source install/setup.bash
# Or add permanently:
echo "source ~/robot_ws/install/setup.bash" >> ~/.bashrc
```

> **Recommended:** switch to Cyclone DDS to fix synchronization issues between the XV-11 driver and SLAM Toolbox:
>
> ```bash
> echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
> ```

---

## Running

### Real Robot PENDING

```bash
# Terminal 1 – bring up robot (ros2_control, controllers, robot description)
ros2 launch robot launch_robot.launch.py
    # includes:  
    # ros2 run xv_11_laser_driver neato_laser_publisher \
        #--ros-args -p port:=/dev/serial/by-id/<your-device-id>
    # ros2 launch mpu9250driver mpu9250driver_launch.py
    # ros2 run imu_filter_madgwick imu_filter_madgwick_node \
        #--ros-args -p use_mag:=false -p publish_tf:=false -p world_frame:=enu
    # ros2 launch robot ekf_imu.launch.py

# Terminal 2 - launch navigation and SLAM tools
    # includes:
    # ros2 launch slam_toolbox online_async_launch.py \
        # slam_params_file:=~/robot_ws/src/robot/config/mapper_params_online_async.yaml
    # ros2 launch robot navigation_launch.py

# Terminal 3 – Teleoperation (optional)
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/cmd_vel_teleop
```

### Simulation (Gazebo Fortress)

```bash
# Terminal 1 – simulated robot
ros2 launch robot launch_sim.launch.py

# Terminal 2 – RViz with the odometry config
rviz2 -d src/robot/config/odom.rviz

# Terminal 3 – keyboard teleoperation
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/cmd_vel_key
```

---

## Useful Commands

```bash
# List available serial devices (stable IDs)
ls -l /dev/serial/by-id/

# Check the TF frame tree
ros2 run tf2_tools view_frames

# Monitor odometry
ros2 topic echo /diff_cont/odom   # encoder crudo (entrada del EKF)
ros2 topic echo /odom             # filtrado del EKF (remapeado de odometry/filtered)

# Check active controllers
ros2 control list_controllers
ros2 control list_hardware_interfaces

# Save SLAM map via service
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/home/pablo/my_map'}"

# Check active ROS middleware
ros2 doctor --report | grep middleware
```

---

## Documentation Index PENDING

| Topic | Notion |
|-------|--------|
| Project overview | [Robot LiDAR 2025](https://www.notion.so/352ce5a2286e801ca74dd87a884d5030) |
| LiDAR XV-11: hardware & ROS 2 driver | [→](https://www.notion.so/35dce5a2286e813eb580c0b12a64881d) |
| SLAM with slam_toolbox | [→](https://www.notion.so/35dce5a2286e8166b38ccc4ba3ce74d2) |
| Navigation: Nav2, AMCL, twist_mux | [→](https://www.notion.so/35dce5a2286e81e3b5fffb86fec89955) |
| IMU MPU-9250: driver, Madgwick, EKF | [→](https://www.notion.so/35dce5a2286e813980cacd81f17b304d) |
| ros2_control & odometry calibration | [→](https://www.notion.so/35dce5a2286e81b1b145e80698665914) |
| ESP32-S3 firmware (motor controller) | [→](https://www.notion.so/35bce5a2286e8088b98bc4f5186d5208) |
| PI controller design | [→](https://www.notion.so/35ace5a2286e8000afc2db601cfca9d2) |
| Incremental encoders (AS5040) | [→](https://www.notion.so/35bce5a2286e8059bb8aecaf62a803b0) |
| Ubuntu & ROS 2 setup | [→](https://www.notion.so/354ce5a2286e802c9a9cf6c45b1142c5) |

---

## License

MIT