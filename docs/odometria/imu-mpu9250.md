---
title: "IMU MPU-9250: Driver y Fusión EKF"
description: "MPU-9250 por I²C (3,3 V) publicando /imu/data_raw a ~100 Hz. El EKF de robot_localization fusiona vx y vyaw de la odometría con vyaw del giróscopo. Madgwick y magnetómetro evaluados pero no usados (interferencia de motores)."
sidebar:
  order: 1
---

| Parámetro | Valor |
| --- | --- |
| Sensor | MPU-9250 por I²C (3,3 V) |
| Tópico | `/imu/data_raw` a ~100 Hz |
| Fusión | EKF de robot_localization: vx y vyaw de la odometría + vyaw del giróscopo |
| Descartados | Madgwick y magnetómetro (interferencia de motores) |

Una **IMU** (Inertial Measurement Unit) es un sensor que mide fuerzas inerciales. En general se compone de un giróscopo de 3 (velocidad angular rad/s), un acelerómetro de 3 ejes (aceleración lineal, incluyendo la gravedad, en g), y un magnetómetro de 3 ejes de (campo magnético terrestre, µT).

> [!NOTE]
> **Modo de uso:** se integra al sistema de odometría para corregir los errores acumulados de los encoders. `robot_localization` (EKF) fusiona la velocidad angular del giróscopo con la odometría y publica un TF `odom→base_link` más confiable.

> [!IMPORTANT]
> **Actualización (informe final, §5.6).** En la configuración definitiva **no se utiliza el filtro Madgwick**: el EKF consume directamente `/imu/data_raw` y de la IMU fusiona únicamente la velocidad angular en Z (vyaw) del giróscopo. El magnetómetro se calibró, pero no se fusiona por la interferencia de los motores; su tópico `/imu/mag` se sigue publicando. Las secciones sobre Madgwick se conservan a modo informativo.

### ¿Por qué incorporar una IMU?

Hasta ahora, la odometría sólo se calcula con los encoders incrementales, pero estos acumulan un error sistemático, cada irregularidad en el suelo o en las ruedas produce desvíos de pose que luego pueden no ser corregidas durante la navegación.
La incorporación de la IMU en la odometría permite corregir estos errores, creando una estimación de posición más confiable y consistente.

### Diagrama de bloques de los sensores y filtros

El módulo IMU se integra entre la odometría y las herramientas de navegación, reemplazando la función de diff_drive_controller, que antes se encargaba de publicar la transformación de frames: odom→base_link. Este rol lo ocupará el paquete robot_localization mediante un nodo de estimación de estado con filtro EKF (Filtro de Kalman Extendido), que fusiona IMU y odom. De esta forma no se requieren cambios en herramientas que usan odometría más adelante: SLAM Toolbox y Nav2.

```yaml
MPU-9250 (hardware)
│  I2C
▼
[mpu9250driver]          → /imu/data_raw  (accel + gyro crudos)
│                        → /imu/mag       (publicado, no fusionado)
│
├<── /diff_cont/odom  (odometría de encoders)
▼
[robot_localization EKF] → /odometry/filtered  +  TF odom→base_link
│
▼
[SLAM Toolbox + Nav2]
```

### Árbol de frames con IMU integrada

Definiciones previas: 

- Qué es [URDF](../fundamentos/diccionario.md)
- Qué es [XACRO](../fundamentos/diccionario.md)
- Qué es [TF2](../fundamentos/diccionario.md)

```
map → odom → base_footprint → base_link → chassis → laser_frame
                                                   → imu_frame
```

Los frames `imu_frame` y `laser_frame` son declarados en el URDF como joints fijos respecto a `chassis`. `robot_state_publisher` los publica estáticamente en `/tf_static`. El EKF se abstrae del MPU-9250, solo consume `sensor_msgs/Imu` y consulta TF2 para saber dónde está ese sensor. Si se cambia la IMU por otro modelo, solo se actualizaría el driver y el URDF.

### Hardware: MPU-9250 / MPU-6500

El MPU-9250 integra el MPU-6500 (giroscopio + acelerómetro) y el magnetómetro AK8963. Se comunica vía I2C o SPI.

| Sensor | Rango |
| --- | --- |
| Giroscopio | ±250, ±500, ±1000, ±2000 °/s |
| Acelerómetro | ±2g, ±4g, ±8g, ±16g |
| Magnetómetro (AK8963) | ±4800 µT |

Todos los ADC tienen resolución de 16 bits. Comunicación por I²C, alimentación de 3,3 V. El magnetómetro es susceptible a interferencias externas y requiere un procedimiento de calibración específico.

Datasheet: 
[https://www.farnell.com/datasheets/1670013.pdf](https://www.farnell.com/datasheets/1670013.pdf) 

## Conceptos y Desarrollo Paso a Paso

### Paso 1: Verificar I2C en la Raspberry Pi 4

```bash
# Ver interfaces I2C habilitadas
ls /dev/i2c*
	# Debe mostrar parecido a /dev/i2c-1

# Instalar herramientas de diagnóstico
sudo apt install i2c-tools

# Verificar que el MPU-9250 está conectado y visible
sudo i2cdetect -y 1
	# Debe mostrar la dirección 0x68 ó 0x69

# Agregar usuario al grupo I2C
sudo usermod -aG i2c $USER
```

### Paso 2: Instalar el driver mpu9250driver

```bash
cd ~/robotLidar/src
git clone https://github.com/pablem/2025-LidarRobot.git

# Dependencia del sistema
sudo apt install libi2c-dev

cd ~/robotLidar
colcon build --packages-select mpu9250driver
source install/setup.bash

# Lanzar el driver
ros2 launch mpu9250driver mpu9250driver_launch.py

# Verificar datos
ros2 topic list
ros2 topic echo /imu/mag
ros2 topic echo /imu/data_raw
ros2 topic hz /imu/data_raw   # debe ser ~100 Hz

# Graficar en tiempo real
ros2 run rqt_plot rqt_plot
	# Topics sugeridos: 
		#/imu/data/linear_acceleration/x  
		#/imu/data/angular_velocity/z
```

#### Modificaciones al driver original

El driver [pablem/2025-LidarRobot](https://github.com/pablem/2025-LidarRobot/commit/75bbd54488f1805de8fa3aadf767db373aec371f) incluye cambios respecto al upstream: [hiwad-aziz/ros2_mpu9250_driver](https://github.com/hiwad-aziz/ros2_mpu9250_driver)

- Se eliminó el cálculo de roll/pitch/yaw basado en acelerómetro y magnetómetro (sin filtrado); el driver no estima orientación.
- El driver ahora publica únicamente datos crudos (`/imu/data_raw`) y el magnetómetro como `/imu/mag`.
- Lectura en bloques (accel + gyro + mag); conversión de unidades a SI; validación del magnetómetro; bias del magnetómetro; corrección de covarianzas; actualización de la calibración (accel/gyro).

> [!NOTE]
> Aclaración: el magnetómetro no puede entrar directo al EKF; habría que integrarlo antes con un filtro de orientación como Madgwick. En la versión final no se fusiona (ver Paso 6).

### Paso 3: Agregar la IMU al XACRO (URDF)

En el URDF, agregar el joint fijo `imu_frame` como hijo de `chassis` con la posición y orientación física real del sensor:

```xml
<joint name="imu_joint" type="fixed">
  <parent link="chassis"/>
  <child link="imu_link"/>
  <origin xyz="0.0 0.0 0.05" rpy="0 0 0"/>
</joint>
<link name="imu_link"/>
```

> [!NOTE]
> La corrección de posición y orientación del sensor en el URDF es crítica: define cómo TF2 transforma las mediciones de `imu_frame` al `base_link`. Un offset incorrecto genera errores sistemáticos en la estimación de pose.

### Paso 4: Filtro Madgwick (imu_tools)

> [!NOTE]
> **Informativo — no utilizado en la configuración final.** Se evaluó durante el desarrollo y luego se eliminó del lanzamiento; en la versión final el EKF toma la velocidad angular directamente de `/imu/data_raw`.

Documentación: https://docs.ros.org/en/ros2_packages/humble/api/imu_filter_madgwick/

Repositorio: https://github.com/CCNYRoboticsLab/imu_tools.git

Algoritmo Madgwick: https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/

El `imu_filter_madgwick` es un filtro que fusiona velocidades angulares, aceleraciones y opcionalmente lecturas magnéticas de una IMU genérica para producir una orientación.

Por otra parte la herramienta da la opción `imu_complementary_filteres` (no usada) que tiene un enfoque novedoso basado en una fusión complementaria. 

El `imu_filter_madgwick` produce una orientación estimada en cuaternión

Un cuaternión se representa como un vector de cuatro números: q = [w, x, y, z] 

w (Escalar): Representa la magnitud de la rotación

x, y, z (Vectorial): Representan el eje en el espacio tridimensional sobre el cual ocurre la rotación.

En nuestro caso, cumple el rol de preprocesador: toma los datos crudos del driver (/imu/data_raw) y entrega una orientación confiable (/imu/data) que luego consume el EKF. Sin este paso, el EKF recibiría una orientación no filtrada.

#### Instalación

```bash
# instalación (en pc y en RP4) 
sudo apt install ros-humble-imu-tools

# Inicialización (para probar en rviz) 
ros2 run imu_filter_madgwick imu_filter_madgwick_node \
  --ros-args \
  -p use_mag:=false \         # primeras pruebas, magnetómetro desactivado
  -p publish_tf:=false \      # evitar conflictos de TF  
  -p world_frame:=enu         # East-North-Up frame, for ground robots
```

En RViz: Add → By topic → /imu/data. Este plugin: rviz_imu_plugin, se instala cuando instalamos  imu_tools en PC.
Se visualiza el eje de coordenadas que rota cuando se mueve el robot:
rojo = x
verde = y
azul = z

### Paso 5: EKF (robot_localization)

Documentación Nav2: [https://docs.nav2.org/setup_guides/odom/setup_robot_localization.html#configuring-robot-localization](https://docs.nav2.org/setup_guides/odom/setup_robot_localization.html#configuring-robot-localization)

Wiki, config (Melodic) [https://docs.ros.org/en/melodic/api/robot_localization/html/index.html](https://docs.ros.org/en/melodic/api/robot_localization/html/index.html)

Ejemplo (Jazzy) [https://automaticaddison.com/sensor-fusion-and-robot-localization-using-ros-2-jazzy/](https://automaticaddison.com/sensor-fusion-and-robot-localization-using-ros-2-jazzy/)

robot_localization es un paquete que contiene nodos de estimación de estado, implementaciones de estimadores no lineales para robots en espacio 3D. Contiene ekf_localization_node y ukf_localization_node, y soporta fusión de un número arbitrario de sensores sin restricción en la cantidad de fuentes de entrada.

En nuestra implementación recibe dos entradas y produce una salida:
Entrada: /diff_cont/odom (velocidades de encoders) + /imu/data_raw (velocidad angular Z)
Salida: /odometry/filtered (pose estimada suavizada) + TF odom→base_link

#### Conceptos clave

**5.1 Filtro de Kalman.** Algoritmo que, a partir de una serie de mediciones tomadas a lo largo del tiempo —con ruido estadístico e inexactitudes— produce estimaciones de variables desconocidas más precisas que las obtenidas a partir de una sola medición, estimando una distribución de probabilidad conjunta sobre las variables en cada paso de tiempo. Su estructura formal es un conjunto de ecuaciones matemáticas que proveen una solución recursiva y computacionalmente eficiente al método de mínimos cuadrados en sistemas dinámicos. [Wikipedia](https://en.wikipedia.org/wiki/Kalman_filter)[ScienceDirect](https://www.sciencedirect.com/topics/computer-science/kalman-filter)

---

**5.2 EKF (Extended Kalman Filter).** Variante no lineal del filtro de Kalman que linealiza en torno a una estimación de la media y la covarianza actuales. En los casos donde los modelos de transición están bien definidos, el EKF ha sido considerado el estándar de facto en la teoría de estimación de estado no lineal. Para adaptar el filtro de Kalman a sistemas no lineales, el EKF adopta técnicas del cálculo —concretamente, expansiones en series de Taylor multivariadas— para linealizar el modelo en torno a un punto de operación. [WikipediaWikipedia](https://en.wikipedia.org/wiki/Extended_Kalman_filter)

---

**5.3 UKF (Unscented Kalman Filter)** Extensión del filtro de Kalman para sistemas no lineales en la que se emplea un conjunto de puntos sigma ponderados para aproximar la distribución de la variable de estado aleatoria. El algoritmo genera primero un conjunto de valores de estado denominados puntos sigma, que capturan la media y la covarianza de las estimaciones de estado, y los propaga a través de las funciones de transición y de medición para obtener un nuevo conjunto de puntos transformados, de cuya media y covarianza se extraen las estimaciones finales del estado

#### (5.) Instalación y configuración (nodo EKF)

```bash
sudo apt install ros-humble-robot-localization
```

Ejemplo config.: [https://github.com/cra-ros-pkg/robot_localization/blob/humble-devel/params/ekf.yaml](https://github.com/cra-ros-pkg/robot_localization/blob/humble-devel/params/ekf.yaml)

Ver video robot_localization RosCon2015 [https://vimeo.com/142624091](https://vimeo.com/142624091)

#### Configuración del ekf.yaml

```yaml
# Archivo: /robot/config/ekf.yaml
ekf_filter_node:
  ros__parameters:
    frequency: 30.0
    sensor_timeout: 0.1
    two_d_mode: true          # robot plano: se ignora todo lo relativo a Z
    publish_tf: true
    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom         # estimación local continua, sin correcciones globales

    # Fuente 1: odometría de encoders
    odom0: /diff_cont/odom
    odom0_config: [false, false, false,   # x, y, z
                   false, false, false,   # roll, pitch, yaw
                   true,  false, false,   # vx   <- velocidad lineal medida
                   false, false, true,    # vyaw <- velocidad angular medida
                   false, false, false]   # ax, ay, az
    odom0_pose_rejection_threshold: 5.0
    odom0_twist_rejection_threshold: 5.0

    # Fuente 2: unidad inercial
    imu0: /imu/data_raw
    imu0_config: [false, false, false,
                  false, false, false,    # orientación absoluta no fusionada
                  false, false, false,
                  false, false, true,     # vyaw <- giróscopo
                  false, false, false]
    imu0_remove_gravitational_acceleration: true
```

Decisiones de configuración:

- **Modo bidimensional:** el robot opera sobre superficie plana.
- **Fusión de velocidades y no de poses:** de la odometría se toman la velocidad lineal longitudinal y la velocidad angular, no la pose acumulada.
- **Redundancia sobre vyaw:** la velocidad angular se toma de ambas fuentes para mejorar la estimación.
- **Umbrales de rechazo (Mahalanobis):** agregados luego de probar el robot, para descartar mediciones anómalas de odometría que originaban saltos en la pose y, por propagación, discontinuidades en el mapa.
- **Orientación absoluta no fusionada:** de la IMU se toma exclusivamente la velocidad angular (ver Paso 6).

#### Ajuste en diff_drive_controller

El EKF debe ser el único publisher de `odom→base_link`. Desactivar la TF del controlador:

```yaml
# en my_controllers.yaml
diff_cont:
  enable_odom_tf: false
```

#### Consideraciones prácticas

Sin Madgwick ni magnetómetro, de la IMU sólo se fusiona la velocidad angular Z del giróscopo; roll/pitch/yaw y aceleraciones quedan en false.

Se configuraron umbrales de rechazo (`odom0_pose/twist_rejection_threshold`) para descartar mediciones anómalas que causaban saltos en el mapa.

#### Covarianzas del EKF

La **covarianza** representa la incertidumbre esperada de cada medición. La diagonal de la matriz de covarianza contiene las varianzas (σ²) de cada variable:

La covarianza representa la incertidumbre (error esperado) asociada a una variable aleatoria.
Para el caso de múltiples variables, se utiliza una matriz de covarianza, cuya diagonal contiene las varianzas (σ²) de cada variable y los elementos fuera de la diagonal representan correlaciones entre variables

- σ² pequeña → medición confiable (bajo ruido).
- σ² grande → medición ruidosa (alta incertidumbre).

##### Valores típicos para robot diferencial interior:

En el sistema de fusión sensorial con EKF, estas covarianzas se configuran en distintos niveles:

En los sensores (por ejemplo, IMU), mediante: orientation_covariance, angular_velocity_covariance, linear_acceleration_covariance. Y en el filtro EKF, mediante: process_noise_covariance. Esta última representa la incertidumbre del modelo dinámico del robot, es decir, el error que se asume en cada paso de predicción.

Es decir que, El EKF opera en dos etapas: Predicción, basada en el modelo del robot; y Corrección, basada en las mediciones de los sensores

La matriz process_noise_covariance define cuánto error se introduce en la etapa de predicción:

Si el modelo es **poco preciso**, se utilizan valores **grandes**, lo que incrementa la incertidumbre de la predicción y hace que el filtro confíe más en las mediciones. Si el modelo es preciso, se utilizan valores pequeños, lo que hace que el filtro confíe más en la predicción.

Para un robot diferencial en entornos interiores, se emplean típicamente valores como:

(yaw: orientación, vyaw: velocidad angular, ax: aceleración)

| Variable | Valor | Razón |
| --- | --- | --- |
| x, y | ~0.05 | Incertidumbre moderada en posición |
| yaw | ~0.06 | Mayor incertidumbre en orientación |
| vx | ~0.025 | Velocidad longitudinal relativamente confiable |
| vy | ~0 | Despreciable en robot diferencial |
| ax | bajo/moderado | Ruido de la IMU |

La `process_noise_covariance` del EKF define cuánto error se introduce en la etapa de predicción:

- **IMU con covarianza demasiado pequeña**: la IMU domina; se observa jitter en orientación.
- **IMU con covarianza demasiado grande**: domina la odometría; deriva acumulativa en giros.
- **Convergencia lenta**: aumentar el valor en `process_noise_covariance` para la variable problemática.

### Paso 6: Calibración del Magnetómetro

Hasta el momento se intentó tener una orientación lo más precisa posible mediante una odometría corregida (correcciones en dimensiones, separación de ruedas) y fusión mediante estimación de modelos y los sensores de giroscopio y acelerómetro de la IMU. Sin embargo, no se cumple con la precisión esperada, el robot llega a la base con una leve desviación; por lo que incorporaremos el magnetómetro de la IMU como una fuente adicional de información. A diferencia del giroscopio y la odometría, el magnetómetro permite referenciar la orientación respecto al campo magnético terrestre, evitando la acumulación de errores. Desventaja: alta sensibilidad a motores DC, corrientes eléctricas y estructuras metálicas cercanas.

- **Error hard-iron**: interferencia constante que actúa como offset del campo magnético terrestre.
- **Error soft-iron**: deformación del campo por materiales ferromagnéticos cercanos.

#### Proceso de calibración (realizado con motores apagados)

**Paso 1 – Adquisición de datos:**

```bash
ros2 bag record /imu/mag -o mag_calib
# Mover el robot en todos los ejes (no solo yaw) durante 30–60 s
```

**Paso 2 – Cálculo del offset:**

```bash
python3 mag_calibration.py
```

Salida del script:

```
--- MAGNETOMETER CALIBRATION ---
Samples: 7797

Bias (Tesla):
mag_bias_x: 2.86875e-05
mag_bias_y: 2.4583984375000002e-05
mag_bias_z: -2.5927734374999997e-05
```

**Paso 3 – Implementación en el driver:**

```yaml
# mpu9250driver config
mpu9250driver:
  ros__parameters:
    mag_bias_x: 2.86875e-05
    mag_bias_y: 2.4583984375e-05
    mag_bias_z: -2.5927734375e-05
```

En el launcher del filtro Madgwick y en el EKF, activar `use_mag: true` para habilitar el yaw absoluto.

#### Decisión final sobre el magnetómetro

En condiciones de operación, la conmutación de los motores introduce perturbaciones variables que un sesgo constante no compensa. Verificado este comportamiento, se resolvió **no incorporar el magnetómetro como fuente del filtro**: el tópico `/imu/mag` se publica y queda disponible, pero la orientación absoluta no se fusiona (por lo tanto, el paso anterior de `use_mag: true` no se aplicó). Aun así, la sola inclusión del giróscopo mejora notablemente la estimación de la pose.

Los valores de sesgo quedaron declarados en `mpu9250driver/params/mpu9250.yaml`.

## Referencias

- [github.com/hiwad-aziz/ros2_mpu9250_driver](https://github.com/hiwad-aziz/ros2_mpu9250_driver)
- [github.com/cra-ros-pkg/robot_localization](https://github.com/cra-ros-pkg/robot_localization)
- [github.com/CCNYRoboticsLab/imu_tools](https://github.com/CCNYRoboticsLab/imu_tools)
- [x-io.co.uk/open-source-imu-and-ahrs-algorithms](https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/)
- [docs.nav2.org/setup_guides/odom/setup_robot_localization.html](https://docs.nav2.org/setup_guides/odom/setup_robot_localization.html)
- [docs.ros.org/en/ros2_packages/humble/api/imu_filter_madgwick](https://docs.ros.org/en/ros2_packages/humble/api/imu_filter_madgwick/)
- [docs.ros.org/en/melodic/api/robot_localization/html/index.html](https://docs.ros.org/en/melodic/api/robot_localization/html/index.html)
- [automaticaddison.com/sensor-fusion-and-robot-localization-using-ros-2-jazzy](https://automaticaddison.com/sensor-fusion-and-robot-localization-using-ros-2-jazzy/)
- [github.com/cra-ros-pkg/robot_localization/blob/humble-devel/params/ekf.yaml](https://github.com/cra-ros-pkg/robot_localization/blob/humble-devel/params/ekf.yaml)
- [vimeo.com/142624091](https://vimeo.com/142624091)
- [en.wikipedia.org/wiki/Kalman_filter](https://en.wikipedia.org/wiki/Kalman_filter)
- [en.wikipedia.org/wiki/Extended_Kalman_filter](https://en.wikipedia.org/wiki/Extended_Kalman_filter)
