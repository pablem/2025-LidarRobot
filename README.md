### Folders:
- **robot/ (package)**  
  Launcher. Descripción URDF/Xacro. Configs: ros2_control, mapper, nav2, rviz2.
  Fork from: https://github.com/joshnewans/my_bot

- **diffdrive_arduino/ (node)**
  HardwareInterface (o UserInterface) ros2_control → Arduino/ESP32. 
  Publica encoders y velocidades de cada motor. Recibe comandos del controlador diferencial.
  Diseñado para usar con el controlador diff_drive_controller del framework ros2_control

  Copied from: https://github.com/joshnewans/diffdrive_arduino
  
  Fork from: https://github.com/ros-controls/ros2_control_demos/tree/master/example_2

- **serial/ (lib)**  
  Biblioteca C++ para comunicación serial (asíncrona/síncrona) utilizada por diffdrive_arduino. Copied from: https://github.com/joshnewans/serial
  
- **lidar_driver**

- **firmware**


### Dependencias ROS2
```bash
sudo apt update
sudo apt install -y \
  ros-humble-xacro \
  ros-humble-ros2-control
```

### Clonar y compilar
```bash
# crear workspace
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src

# es un repositorio único (contiene tódos los paquetes y drivers), es la carpeta "src/" del proyecto
git clone https://github.com/pablem/2025-LidarRobot.git
mv 2025-LidarRobot src
cd ~/robot_ws

# compilar
colcon build --symlink-install

# activar el entorno
source install/setup.bash

```