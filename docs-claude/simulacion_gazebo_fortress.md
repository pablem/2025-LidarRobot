## Gazebo Fortress e Ignition

Se usa **Gazebo Fortress** (= **Ignition Gazebo 6**). Un poco de historia para
ubicarse:

- **Gazebo Classic** (Gazebo 11): el de siempre, monolítico. Llegó a *end-of-life
  en enero 2025* → por eso migramos.
- **Ignition Gazebo**: la reescritura modular de OSRF. Fortress (2021) es una
  versión LTS de esa era. Sus binarios y plugins todavía llevan el prefijo
  `ignition`/`ign` (ej: comando `ign gazebo`, plugins `ignition::gazebo::...`).
- Más adelante "Ignition" se **renombró a "Gazebo"** (gz) — Garden, Harmonic, etc.
  usan el prefijo `gz`. Eso genera confusión: **Fortress es la última que conserva
  el nombre/CLI `ign`**.

Para **ROS 2 Humble**, Fortress es el emparejamiento **oficial** y el que se instala
sin agregar repos extra. (Harmonic existe y dura más como LTS, pero requería repos
de OSRF y meta-paquetes que no estaban disponibles.)

### Instalación

```bash
sudo apt install ros-humble-ros-gz ros-humble-gz-ros2-control
```

- `ros-humble-ros-gz` → meta-paquete que trae `ros_gz_sim` (lanzar/spawnear),
  `ros_gz_bridge` (puente de topics gz↔ROS), `ros_gz_image`, `ros_gz_interfaces`,
  y arrastra **Fortress** (`libignition-gazebo6`).
- `ros-humble-gz-ros2-control` → el plugin que conecta `ros2_control` con Gazebo.

---

## Características y pequeña guía

- **Arquitectura**: servidor (física + sensores) + GUI unificados bajo el comando
  `ign gazebo`. Headless = `-s` (solo server); `-r` = correr al iniciar (sin darle
  play a mano); `-v4` = verbose.
- **Transport propio**: Gazebo usa **gz-transport**, su propio middleware, **NO DDS
  de ROS**. Por eso los sensores y el reloj **no aparecen en ROS automáticamente**:
  hace falta `ros_gz_bridge` (ver abajo). El control sí entra por ROS porque
  `gz_ros2_control` levanta un `controller_manager` ROS adentro de Gazebo.
- **GUI** (paneles útiles):
  - *Entity tree* — árbol de modelos/links del mundo.
  - *Component inspector* — pose, inercia, etc. de la entidad seleccionada.
  - Barra inferior: **play/pause/step** de la simulación y reloj (RTF, sim time).
  - Botones de *view*: colisiones, inercias, frames, wireframe (útil para depurar
    geometría, p. ej. ver si el láser roza el chasis).

---

## El papel de `ros2_control.xacro` vs `gazebo_control.xacro`

Estos dos archivos son el corazón de la paridad sim/real.

**`ros2_control.xacro`** — define el bloque `<ros2_control>` (joints + interfaces de
comando/estado) y, sobre todo, **qué plugin de hardware** se usa. Se intercambia con
un arg `sim_mode`:

```xml
<xacro:unless value="$(arg sim_mode)">
    <plugin>diffdrive_arduino/DiffDriveArduinoHardware</plugin>   <!-- robot real -->
</xacro:unless>
<xacro:if value="$(arg sim_mode)">
    <plugin>ign_ros2_control/IgnitionSystem</plugin>              <!-- simulación -->
</xacro:if>
```

Los `<joint>` y sus interfaces son **idénticos** en ambos modos: por eso
`diff_cont` (diff_drive_controller) y `joint_broad` y `my_controllers.yaml` se
reutilizan tal cual.

**`gazebo_control.xacro`** — se incluye **solo en sim** (`<xacro:if sim_mode>`).
Carga el *system plugin* que mete un `controller_manager` **dentro** de Gazebo,
apuntando a la misma config de controllers:

```xml
<gazebo>
    <plugin filename="ign_ros2_control-system"
            name="ign_ros2_control::IgnitionROS2ControlPlugin">
        <parameters>$(find robot)/config/my_controllers.yaml</parameters>
    </plugin>
</gazebo>
```

En una frase: **`ros2_control.xacro` decide el "hardware" (real vs sim);
`gazebo_control.xacro` arranca la maquinaria de control adentro de Gazebo.**

> La fricción del caster (`<gazebo reference="caster_wheel"><mu1>`) vive en
> `robot_core.xacro` junto al link; sin ella la rueda loca esférica arrastra y
> bloquea los giros.

---

## Los plugins de `imu.xacro` y `lidar.xacro`

No son plugins de ROS: son **sensores SDF nativos de Gazebo**. El *system plugin*
`Sensors` del mundo los ejecuta y publica sus datos a **gz-transport** (no a ROS).

```xml
<!-- lidar.xacro -->
<sensor name="laser" type="gpu_lidar">
    <topic>scan</topic>
    <ignition_frame_id>laser_frame</ignition_frame_id>
    ...
</sensor>

<!-- imu.xacro -->
<sensor name="imu_sensor" type="imu">
    <topic>imu/data_raw</topic>
    <ignition_frame_id>imu_frame</ignition_frame_id>
</sensor>
```

- `type="gpu_lidar"` / `type="imu"` → sensores que provee Fortress.
- `<topic>` → en qué topic de **gz** publican.
- `<ignition_frame_id>` → fija el `frame_id` del mensaje (clave para que SLAM/Nav2
  encuentren la TF correcta).
- Para que esos datos lleguen a ROS, el **bridge** los traduce (`scan` → `/scan`,
  `imu/data_raw` → `/imu/data_raw`).

---

## Cómo llegan los datos a ROS: el bridge

```
Gazebo (gz-transport)                          ROS 2 (DDS)
  /clock ───────────┐
  scan   ───────────┼──►  ros_gz_bridge  ──►   /clock  /scan  /imu/data_raw
  imu/data_raw ─────┘     (parameter_bridge)

  gz_ros2_control (controller_manager ROS) ──► /diff_cont/odom, /joint_states, cmd_vel
```

- **Sensores y reloj**: van por el bridge. El `/clock` es imprescindible para que
  todos los nodos ROS con `use_sim_time:=true` usen el tiempo de simulación.
- **Control y odometría**: NO necesitan bridge — `gz_ros2_control` ya expone
  `diff_cont`/`joint_broad` como nodos ROS normales. `cmd_vel` entra directo al
  controller. El EKF y twist_mux corren nativos en ROS, igual que en el real.

---

## Qué cambios hicieron falta en el proyecto

Conceptos (no exhaustivo — git tiene el detalle):

1. **Arg `sim_mode`** en `robot.urdf.xacro` / `ros2_control.xacro` para intercambiar
   el plugin de hardware (real ↔ `IgnitionSystem`) e incluir `gazebo_control.xacro`
   solo en sim.
2. **Sensores SDF** en `lidar.xacro` e `imu.xacro` (gpu_lidar / imu) publicando a
   gz-transport.
3. **Mundo Fortress** (`worlds/obstacles_gz.sdf`, SDF 1.8) con los *system plugins*
   obligatorios: `physics`, `sensors`, `scene-broadcaster`, `user-commands`. Sin
   ellos no hay física ni sensores. Se hizo self-contained (sin modelos de Fuel)
   para que cargue offline.
4. **`launch_sim.launch.py`**: arranca `gz_sim` con el mundo, spawnea el robot con
   `ros_gz_sim create -topic robot_description`, levanta el `ros_gz_bridge`
   (clock/scan/imu), los spawners de controllers, twist_mux y el EKF — todo con
   `use_sim_time:=true`.
6. **`use_sim_time` parametrizable** en `slam_nav.launch.py` y `explore.launch.py`
   (default `false`; en sim se pasa `:=true`) para que SLAM, Nav2 y explore_lite
   compartan el reloj de simulación.

---

## Cómo correr

```bash
# 1. Simulación (Gazebo + robot + controllers + bridge + EKF)
ros2 launch robot launch_sim.launch.py

# 2. SLAM + Nav2 con reloj de simulación
ros2 launch robot slam_nav.launch.py use_sim_time:=true

# 3. Exploración autónoma
ros2 launch robot explore.launch.py use_sim_time:=true
```

