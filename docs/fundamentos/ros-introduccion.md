---
title: "ROS: Introducción extendida"
description: "Robot Operating System: conceptos, arquitectura y uso práctico."
sidebar:
  order: 2
---

> [!NOTE]
> Apuntes + documentación inicial generada con materiales del workshop “Practical Introduction to ROS, 3D Models and Embedded Integration” dictado por Mg. Ing. Pablo Slavkin durante el SASE 2024 en la FACET, UNT.

## Recursos

https://docs.ros.org/en/humble/Tutorials.html
https://husarion.com/tutorials/ros2-tutorials/1-ros2-introduction/
https://design.ros2.org/

## 1. ¿Qué es ROS?

ROS es la sigla para **Robotics Operating System**. Es un “meta-OS” open source cuyo objetivo es hacer el desarrollo de software para robots más rápido, fácil y versátil.

> [!NOTE]
> **Nota**: no es un sistema operativo en el sentido tradicional, no gestiona directamente el hardware ni el scheduling del procesador. En general, se instala sobre un OS como Linux/Ubuntu.

### ¿Por qué ROS?

> [!NOTE]
> Cada vez que empezamos un nuevo proyecto perdemos tiempo reescribiendo código y resolviendo problemas que otros ya resolvieron varias veces.

El software de robots tiende a ser complejo: se encarga de controlar sensores de distancia, cámaras, motores, unidades de control de movimiento, algoritmos de visión artificial, módulos de planificación de trayectorias, etc; ROS permite que cada una de estas responsabilidades sea manejada por un programa independiente (llamado nodo), y define mecanismos estándar para que todos esos programas se comuniquen entre sí, ya sea que estén corriendo en la misma máquina o en distintas computadoras conectadas en red (Sistemas Distribuidos). Cada componente puede ser desarrollado, probado y reemplazado de forma aislada.

Actualmente, la comunidad ROS incluye tanto entusiastas, estudiantes como empresas, por lo tanto existe abundante documentación, tutoriales, foros y paquetes actualizados, como drivers para dispositivos comerciales y stacks de navegación.  

### ¿Por qué no ROS?

Instalación Compleja: Para nuevos usuarios se recomienda trabajar en entorno oficial: Ubuntu, por su compatibilidad oficial y menor complejidad. En otros sistemas operativos a veces es necesario recurrir a contenedores (Docker, Distrobox) o máquinas virtuales.

**Retrocompatibilidad**: Las distintas distribuciones de ROS 2 pueden introducir cambios que requieren adaptar el código existente. Por más que sean LTS, el soporte dura relativamente poco y hay que pasar a distros más recientes.

## 2. Arquitectura y Componentes

La arquitectura de ROS se organiza alrededor del concepto de Grafo Computacional (ROS Computational Graph). Este grafo representa la red de procesos en ejecución y sus canales de comunicación. Los elementos principales son: Nodos, Tópicos, Servicios, Acciones, Mensajes y el Máster (en ROS 1) o el middleware basado en DDS (en ROS 2).

### 2.1. Nodos

Un nodo de ROS es un proceso ejecutable que realiza una tarea específica de computación. La filosofía de ROS es mantener cada nodo con una responsabilidad bien definida y acotada. En un sistema robótico típico podría haber, por ejemplo:

- Un nodo que lee los datos del sensor LiDAR y los publica.
- Un nodo que procesa esos datos para detectar obstáculos.
- Un nodo que controla los motores de las ruedas.
- Un nodo que implementa el algoritmo de navegación.
- Un nodo que monitorea la temperatura de la batería.

Cada nodo está escrito utilizando alguna de las librerías cliente de ROS (rclpy para Python, rclcpp para C++, etc.) y se registra en el sistema al iniciarse. Los nodos pueden publicar mensajes, suscribirse a tópicos, ofrecer servicios y utilizar servicios de otros nodos.

Una ventaja clave de esta separación es la tolerancia a fallos: si un nodo cae, los demás pueden continuar funcionando. También facilita el debugging, ya que se puede inspeccionar la entrada y salida de cada nodo de forma independiente.

### 2.2. ROS Master (ROS 1) y Discovery DDS (ROS 2)

En ROS 1, el Master es un proceso centralizado que actúa como registro y punto de conexión para todos los nodos. Cuando un nodo se inicia, se registra en el Máster indicando qué tópicos publica y cuáles consume. Cuando un publisher y un subscriber coinciden en un tópico, el Máster les informa sus respectivas direcciones de red para que puedan establecer una conexión directa.

El Máster también gestiona el Servidor de Parámetros, donde los nodos pueden almacenar y recuperar valores de configuración en tiempo de ejecución.

En ROS 2, el Máster centralizado desapareció. En su lugar se usa DDS (Data Distribution Service), un estándar industrial para comunicación distribuida. DDS implementa un mecanismo de descubrimiento automático: los nodos se encuentran entre sí mediante multicast UDP, sin necesidad de un proceso central. Esto facilita el despliegue en redes con múltiples máquinas.

(El User Datagram Protocol (UDP) es un protocolo de transporte de red rápido y ligero que envía datos (datagramas) sin establecer una conexión previa, priorizando la velocidad sobre la fiabilidad. A diferencia de TCP, no garantiza el orden ni la entrega de paquetes)

### 2.3. Mensajes

Los nodos se comunican intercambiando mensajes. Un mensaje es una estructura de datos tipada, similar a un struct de C. Se definen en archivos con extensión .msg usando un lenguaje de descripción simple. Por ejemplo, un mensaje de temperatura podría ser:

`float32 temperature`

`string unit`

`builtin_interfaces/Time stamp`

ROS provee una amplia biblioteca de mensajes estándar para los tipos de datos más comunes en robótica: poses y transformaciones 3D (geometry_msgs), imágenes (sensor_msgs/Image), nubes de puntos (sensor_msgs/PointCloud2), mensajes de texto (std_msgs/String), entre otros.

Cuando se requiere un tipo de dato que no existe en la biblioteca estándar, se pueden crear mensajes custom. Esto implica definir el archivo .msg dentro de un paquete ROS y compilarlo para que el sistema genere el código necesario en el lenguaje de programación utilizado.

### 2.4. Tópicos

Los tópicos son los canales de comunicación de ROS. Implementan un patrón publisher/subscriber asíncrono: un nodo que produce datos los publica en un tópico, y cualquier nodo interesado puede suscribirse a ese tópico para recibirlos.

Los tópicos se identifican por un nombre en formato de ruta, como /temperatura, /velocidad_rueda_izquierda o /camara/imagen_raw. Se recomienda usar nombres descriptivos que dejen claro qué tipo de dato transportan.

Características clave de los tópicos:

- Un tópico puede tener múltiples publishers y múltiples subscribers al mismo tiempo. Es una comunicación de tipo many-to-many.
- Los publishers y subscribers están completamente desacoplados: un publisher no necesita saber si alguien está suscripto, y un subscriber no necesita saber quién publica.
- La comunicación es asíncrona: el publisher envía datos cuando los tiene disponibles, y cada subscriber los procesa en su propio ritmo mediante callbacks.
- Un mismo nodo puede publicar en múltiples tópicos y suscribirse a múltiples tópicos simultáneamente.

### 2.5. Servicios

Los servicios implementan un patrón de comunicación request/reply sincrónico. A diferencia de los tópicos, donde el publisher envía datos sin esperar respuesta, en un servicio un nodo cliente envía una solicitud y espera hasta recibir una respuesta del nodo servidor.

Los servicios se definen en archivos .srv, que especifican la estructura tanto del request como del reply, separados por tres guiones:

## Archivo: SumarEnteros.srv

`int32 a`

`int32 b`

`---`

`int32 resultado`

Características importantes:

- Solo puede haber un “service server” por servicio. Si dos nodos intentan proveer el mismo servicio, puede causar comportamiento indefinido.
- La llamada es bloqueante por defecto: el cliente espera hasta que el servidor responde (aunque existen llamadas asíncronas).
- Son adecuados para operaciones discretas: cambiar un parámetro, iniciar/detener un proceso, obtener el estado actual, ejecutar un movimiento puntual, etc.

### 2.6. Acciones

Las acciones son el mecanismo de ROS para manejar tareas de larga duración que requieren feedback continuo y la posibilidad de cancelación. Están construidas sobre tópicos y servicios.

El flujo de una acción es el siguiente:

- El action client envía un goal (objetivo) al action server. Por ejemplo: "navegar hasta las coordenadas (3.5, 2.1)" o "mover el brazo hasta la posición X".
- El action server acepta o rechaza el goal.
- Durante la ejecución, el server envía feedback periódico al client a través de un tópico: porcentaje completado, distancia restante, etc.
- Una vez completada la tarea (o si ocurre un error), el server envía el resultado final al client mediante un servicio.
- En cualquier momento, el client puede cancelar la acción enviando una solicitud de cancelación.

### 2.7. Parámetros

Los parámetros en ROS son valores de configuración asociados a un nodo específico. Permiten ajustar el comportamiento del nodo sin necesidad de modificar el código ni reiniciar el proceso (en muchos casos). Se pueden leer y escribir desde la línea de comandos o desde otros nodos en tiempo de ejecución.

Para listar todos los parámetros de los nodos en ejecución:

`ros2 param list`

Para obtener o modificar un parámetro específico:

`ros2 param get /mi_nodo velocidad_maxima`

`ros2 param set /mi_nodo velocidad_maxima 0.5`

### 2.8. Librerías Cliente

Las librerías cliente son el punto de acceso desde el código del desarrollador a todas las funcionalidades de ROS. Permiten crear nodos, publicar y suscribirse a tópicos, crear servicios y clientes de servicios, y manejar acciones.

La base de todas las librerías es rcl (ROS Client Library), escrita en C. Sobre esta base se construyen los wrappers para otros lenguajes:

- rclcpp: librería oficial para C++. Ofrece el mejor rendimiento y es más utilizada en aplicaciones de producción. (más adelante: drivers)
- rclpy: librería oficial para Python. Más fácil de usar y con acceso a todo el ecosistema de Python (numpy, scipy, etc.). Es más utilizada prototipado.
- Librerías de la comunidad: rclc (C puro, usado en microcontroladores vía micro-ROS), rclrs (Rust), rcljava (Java/Android), rcldotnet (.NET), rclnodejs (Node.js).

> [!NOTE]
> Las bibliotecas cliente están disponibles en varios lenguajes de programación para que los usuarios puedan escribir código ROS 2 en el lenguaje que mejor se adapte a su aplicación. Por ejemplo, es posible que prefiera escribir herramientas de visualización en Python porque acelera las iteraciones de creación de prototipos, mientras que para las partes de su sistema que se centran en la eficiencia, los nodos podrían implementarse mejor en C.

[Client libraries — ROS 2 Documentation: Lyrical documentation](https://docs.ros.org/en/lyrical/Concepts/Basic/About-Client-Libraries.html?utm_source=copilot.com#id1)

### 2.9. tf2: Sistema de Transformaciones

En ROS, los distintos sensores y componentes del robot tienen cada uno su propio sistema de coordenadas: llamado Frame. Por ejemplo, el Lidar genera puntos con respecto a su cetro de rotación (no a partir del centro del mapa). Para que los algoritmos funcionen correctamente, es necesario saber en todo momento cómo transformar una posición expresada en un frame a cualquier otro frame.

tf2 (Transform Library 2) es un sistema que resuelve este problema. Mantiene un árbol de transformaciones entre todos los frames del robot, actualizándolas en tiempo real a medida que el robot se mueve y sus articulaciones cambian de posición. Cualquier nodo puede consultarle a tf2,  por ejemplo: “cuál es la pose del frame base_link respecto al frame map en este instante”

Ejemplo de árbol de transformaciones:

```python
map
  └─ odom
       └─ base_link
            ├─ base_laser        (LiDAR)
            ├─ camera_link       (cámara)
            ├─ wheel_left
            └─ wheel_right
```

En este ejemplo, el driver del robot publica la transformación odom → base_link a medida que la odometría avanza. El robot_state_publisher publica las transformaciones entre los links del URDF según el estado actual de las articulaciones.

## 3. Interfaz de Línea de Comandos (CLI)

ROS 2 provee una interfaz de línea de comandos a través del comando ros2.

### 3.1. Inicialización del entorno

Antes de poder usar cualquier comando ros2, es necesario inicializar el entorno en cada terminal. Esto se hace ejecutando:

```bash
source /opt/ros/{version}/setup.bash
```

Donde {version} se reemplaza por el nombre de la distribución instalada (jazzy, humble, iron, etc.) y la extensión depende del shell utilizado (.bash, .zsh, .sh, o .ps1 en Windows). 

Para no tener que ejecutar este comando en cada terminal, se puede agregar al archivo de inicialización del shell (~/.bashrc o ~/.zshrc).

### 3.2. Gestión de Paquetes

```bash
ros2 pkg list                    # Lista todos los paquetes instalados
ros2 pkg executables             # Lista todos los ejecutables disponibles
ros2 pkg executables robotLidar  # Ejecutables de un paquete específico
ros2 run {paquete} {ejecutable}  # Inicia un ejecutable
ros2 run robotLidar launch_robot.launch.py
```

### 3.3. Nodos

```bash
ros2 node list                   # Lista los nodos en ejecución
ros2 node info /nombre_nodo      # Información detallada de un nodo

# Renombrar un nodo al iniciarlo:
ros2 run turtlesim turtlesim_node --ros-args --remap __node:=mi_tortuga
```

### 3.4. Tópicos

```bash
ros2 topic list                  # Lista los tópicos activos
ros2 topic list -t               # Lista tópicos con el tipo de mensaje
ros2 topic echo /nombre_topico   # Muestra los mensajes en tiempo real
ros2 topic info /nombre_topico   # Información sobre publicadores y suscriptores
ros2 topic hz /nombre_topico     # Frecuencia de publicación

# Publicar un mensaje manualmente (se repite continuamente):
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"

# Publicar solo una vez:
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"

# Publicar a una frecuencia específica (Hz):
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"
```

### 3.5. Servicios

```bash
ros2 service list                # Lista los servicios disponibles
ros2 service type /nombre_srv    # Tipo de mensaje del servicio

# Llamar un servicio (datos en formato YAML, espacio obligatorio después de ':')
ros2 service call /spawn turtlesim/srv/Spawn '{x: 1.0, y: 5.0, theta: 0.0}'

ros2 service call /kill turtlesim/srv/Kill '{name: turtle1}'
```

### 3.6. Interfaces de mensajes

```bash
ros2 interface show geometry_msgs/msg/Twist  # Estructura de un mensaje
ros2 interface show turtlesim/srv/Spawn      # Estructura de un servicio
ros2 interface list                          # Todos los tipos disponibles
```

### 3.7. Remapeo de tópicos

Remap permite redirigir los tópicos que un nodo usa hacia nombres diferentes sin modificar el código. Esto es muy útil para conectar nodos que usan nombres distintos por defecto:

```bash
ros2 run turtlesim turtle_teleop_key --ros-args --remap turtle1/cmd_vel:=mi_tortuga/cmd_vel
```

### 3.8. tf2

```bash
ros2 run tf2_tools view_frames   # genera un PDF con el árbol completo
ros2 run tf2_ros tf2_echo base_link camera_link  # imprime la transformación en tiempo real

```

## 4. Desarrollo de Paquetes

### 4.1. Workspace

Todo el desarrollo en ROS se organiza dentro de un workspace, que es un directorio con una estructura predefinida. Un workspace puede contener múltiples paquetes. La estructura estándar es:

```bash
mi_workspace/

		src/      <- código fuente de los paquetes
		build/    <- archivos intermedios de compilación
		install/  <- paquetes instalados y scripts de activación
		log/      <- logs de compilación
```

El directorio src es el único que el desarrollador gestiona directamente. Los demás son generados automáticamente por la herramienta de compilación colcon. Para compilar todos los paquetes del workspace se ejecuta colcon build desde la raíz del workspace.

Una vez compilado, es necesario hacer source del script de activación del workspace para que ROS pueda encontrar los paquetes instalados:

```bash
source install/setup.bash
```

Para no tener que hacer esto por cada terminal nueva, se puede agregar al archivo de inicialización del shell (~/.bashrc o ~/.zshrc). 

### 4.2. Creación de un paquete Python

Para crear un nuevo paquete dentro de la carpeta src del workspace:

```bash
cd mi_workspace/src

ros2 pkg create --build-type ament_python --license Apache-2.0 mi_paquete
```

Esto genera la estructura básica del paquete, incluyendo el archivo package.xml (metadatos y dependencias) y setup.py. Los archivos de código Python van dentro de una subcarpeta con el mismo nombre que el paquete.

### 4.3. Estructura de un nodo Python

Un nodo básico en Python con rclpy tiene la siguiente estructura:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MiNodo(Node):
	def __init__(self):
		super().__init__('mi_nodo')
		self.publisher = self.create_publisher(String, '/mi_topico', 10)
		self.timer = self.create_timer(1.0, self.callback_timer)
		
	def callback_timer(self):
		msg = String()
		msg.data = 'Hola desde ROS!'
		self.publisher.publish(msg)
		
def main():
	rclpy.init()
	nodo = MiNodo()
	rclpy.spin(nodo)
	rclpy.shutdown()
```

### 4.4 Archivos Launch

Poner en marcha el robot implica iniciar docenas de nodos de forma simultánea y coordinada, por ejemplo el driver del Lidar, luego el stack de navegación, el publicador del modelo URDF, el servidor de transformaciones, etc. Hacerlo a mano, abriendo una terminal por nodo, sería impracticable. Para esto existen los archivos launch.

Un archivo launch es un script que describe qué nodos iniciar, con qué parámetros, en qué orden, y con qué remapeos de tópicos. 

En ROS 2, los archivos launch se escriben en Python. Tienen extensión .launch.py y residen dentro de la carpeta launch/ de un paquete. La estructura básica es:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='simulador',
        ),
        Node(
            package='mi_paquete',
            executable='controlador',
            name='ctrl',
            remappings=[('/cmd_vel', '/robot/cmd_vel')],
            parameters=[{'velocidad_max': 0.5}],
        ),
    ])
```

Para ejecutar un archivo launch:

`ros2 launch mi_paquete mi_archivo.launch.py`

### 4.5. colcon: Sistema de Build (”compilación”)

colcon (collective construction) es la herramienta oficial de compilación de workspaces en ROS 2. Unifica la compilación de paquetes escritos en distintos lenguajes.

El flujo de trabajo básico con colcon es siempre el mismo: desde la raíz del workspace, se ejecuta:

`colcon build`

Esto compila todos los paquetes en src/ en el orden correcto según sus dependencias, e instala los resultados en install/. 

Después de compilar, es necesario hacer source del entorno para que los cambios sean visibles:

`source install/setup.bash`

Flags que usamos: 

- --symlink-install: en lugar de copiar los archivos Python al directorio install/, crea enlaces simbólicos. Esto significa que los cambios en el código Python son efectivos inmediatamente sin necesidad de recompilar.
- --packages-select <nombre>: compila solo el paquete especificado y sus dependencias directas.

En el caso de necesitar limpiar compilaciones previas:

`rm -rf build/ install/ log/`

## 5. Herramientas del Ecosistema

### 5.1. rviz2

rviz2 (ROS Visualization) es la herramienta de visualización 3D de ROS. Permite visualizar en tiempo real el estado del robot y su entorno a partir de los datos que circulan por los tópicos. Sus usos más comunes incluyen:

- Visualizar el modelo 3D del robot a partir de su descripción URDF.
- Mostrar las lecturas de sensores: nubes de puntos de LiDAR, imágenes de cámara, ejes de movimiento de IMU.
- Visualizar el mapa del entorno construido por un algoritmo de SLAM.
- Mostrar las trayectorias planificadas y el camino recorrido por el robot.
- Depurar transformaciones de coordenadas (tf2, se desarrolla luego)

### 5.2. Gazebo

Gazebo un simulador de robots usado en el ecosistema ROS. Permite probar algoritmos y comportamientos sin necesidad de hardware físico. Sus características principales son:

- Simulación de física realista: masa, inercia, fricción, colisiones, gravedad.
- Modelos de sensores: cámaras, LiDAR, IMU, GPS, etc.
- Integración nativa con ROS a través del paquete ros_gz: los datos de los sensores simulados se publican directamente como mensajes ROS.
- Gran biblioteca de modelos de entornos (interiores, exteriores, industriales) y robots (TurtleBot, Spot, brazos robóticos).

### 5.3. URDF

URDF (Unified Robot Description Format) es el formato estándar de ROS para describir la estructura física de un robot. Es un XML que define:

- Links: los eslabones del robot (cuerpo, ruedas, articulaciones, sensores). Cada link tiene geometría visual, geometría de colisión y propiedades de inercia.
- Joints: las articulaciones que conectan los links. Pueden ser fijas, de revolución, prismáticas, continuas, etc.
- Propiedades físicas: masa, centro de gravedad, tensor de inercia.

El URDF es utilizado tanto por rviz2 para visualizar el robot como por Gazebo para simularlo. También es la base del sistema de transformaciones tf2, que mantiene la relación espacial entre todos los marcos de referencia del robot en tiempo real.

### 5.5. rqt

rqt es una plataforma de herramientas gráficas para ROS construida sobre Qt. Funciona como un dashboard configurable con plugins intercambiables. Los plugins más útiles incluyen:

- rqt_graph: visualiza el grafo de nodos y tópicos, mostrando qué nodos publican y se suscriben a qué tópicos.
- rqt_plot: grafica en tiempo real los valores numéricos publicados en tópicos.
- rqt_console: muestra y filtra los mensajes de log de todos los nodos.
- rqt_service_caller: permite llamar servicios de forma gráfica.
- rqt_topic: permite publicar mensajes en tópicos de forma gráfica.

### 5.6. Foxglove Studio

Foxglove Studio es una herramienta moderna de visualización y debugging para ROS, disponible como aplicación de escritorio y como aplicación web. Es una alternativa más moderna a rqt con una interfaz más amigable. Permite visualizar topics, reproducir bags grabados, y tiene soporte para acceso remoto vía rosbridge.

### 5.7. rosbag2: Grabación y Reproducción

Permite recolectar los datos de los mensajes que circulan por los tópicos.

Para grabar todos los tópicos activos en el momento:

```bash
ros2 bag record -a                          # graba todos los tópicos
ros2 bag record /scan /odom /camera/image   # graba tópicos específicos
ros2 bag record -a -o nombre_del_bag        # especifica nombre del archivo de salida
```

Para reproducir un bag grabado:

```bash
ros2 bag play mi_grabacion/
ros2 bag play mi_grabacion/ --rate 0.5      # reproducir a la mitad de velocidad
ros2 bag play mi_grabacion/ --loop          # reproducir en bucle
```

Para inspeccionar el contenido de un bag sin reproducirlo:
`ros2 bag info mi_grabacion/`

rosbag2 guarda los datos en un directorio que contiene un archivo de base de datos SQLite3 (.db3) con los mensajes serializados y un archivo de metadata YAML con información sobre los tópicos, tipos de mensajes, duración y cantidad de mensajes grabados.

## 6. Distribuciones de ROS

Una distribución (o distro) de ROS es un conjunto versionado y coordinado de paquetes ROS, similar al concepto de distribución en Linux. Cada distribución tiene una fecha de lanzamiento y una fecha de fin de soporte (EOL - End of Life). Las distribuciones con soporte extendido (LTS) son las recomendadas para proyectos productivos.

Se lanza una nueva distribución el 23 de mayo de cada año (fecha del World Turtle Day, que da origen a la mascota de ROS). Las versiones pares de ROS 2 son LTS con 5 años de soporte.

| **Distribución** | **Fecha de lanzamiento** | **Fin de soporte** | **Estado** |
| --- | --- | --- | --- |
| Jazzy Jalisco | 23/05/2024 | 05/2029 | LTS - Actual |
| Iron Irwini | 23/05/2023 | 11/2024 | EOL |
| Humble Hawksbill | 23/05/2022 | 05/2027 | LTS - Soporte activo |

Además de las distribuciones estables, existe Rolling Ridley, una distribución rolling release que recibe actualizaciones continuas. No se recomienda para despliegues estables.

## 7. Arquitectura Interna de Red: DDS como middleware

ROS 2 utiliza DDS (Data Distribution Service) como capa de transporte. DDS es un estándar industrial para comunicación distribuida publish/subscribe en tiempo real. Permite:

- Descubrimiento automático de nodos sin necesidad de un Máster centralizado.
- Comunicación eficiente sobre UDP tanto en la misma máquina (shared memory) como en red.
- Soporte opcional de tiempo real a través de configuraciones de QoS (Quality of Service).

## 8. Integración con Sistemas Embebidos (no se usó)

### 8.1. ros2serial

ros2serial (y su predecesor rosserial) es un protocolo y conjunto de herramientas que permite a microcontroladores como Arduino comunicarse con un sistema ROS a través de un puerto serie (USB, UART). El microcontrolador actúa como un nodo ROS "ligero": puede publicar mensajes en tópicos y suscribirse a tópicos, con la intermediación de un nodo puente que corre en la PC.

Esto es muy útil para robots que tienen una computadora central (Raspberry Pi, PC) corriendo ROS y microcontroladores encargados del control de bajo nivel (motores, sensores analógicos, encoders, etc.).

### 8.2. micro-ROS (uROS)

micro-ROS es la evolución de rosserial y la solución oficial para correr ROS 2 directamente en microcontroladores de recursos limitados (ESP32, STM32, Arduino Due, etc.). A diferencia de rosserial, uROS implementa el cliente ROS directamente en el microcontrolador, comunicándose con el ecosistema ROS 2 a través de un agente que corre en la PC.

El microcontrolador con uROS puede publicar y suscribirse a tópicos, ofrecer servicios y clientes, y en general comportarse como cualquier nodo ROS 2. La biblioteca está optimizada para entornos con memoria y CPU limitadas.

Casos de uso típicos de uROS:

- Control de motores con feedback de encoders publicado como tópico de odometría.
- Lectura de sensores (IMU, temperatura, ultrasónico) y publicación directa como mensajes ROS.
- Implementación de controladores PID en el microcontrolador con setpoints recibidos desde ROS.

## 9. Integración Web: rosbridge y roslibjs (pendiente de usar)

rosbridge es un servidor WebSocket que expone las funcionalidades de ROS a través de una API JSON. Permite que aplicaciones web (y cualquier cliente con soporte de WebSocket) puedan publicar y suscribirse a tópicos, llamar servicios y ejecutar acciones de ROS desde un navegador.

roslibjs es la librería JavaScript complementaria que implementa el protocolo de rosbridge en el navegador. Con roslibjs, un desarrollador web puede construir dashboards de monitoreo, interfaces de control, o herramientas de visualización para robots sin necesidad de instalar ROS.

```jsx
// Ejemplo básico con roslibjs

var ros = new ROSLIB.Ros({ url: 'ws://localhost:9090' });

var topico = new ROSLIB.Topic({

	ros: ros, name: '/temperatura',
	messageType: 'std_msgs/Float32'

});

topico.subscribe(function(msg) {

console.log('Temperatura:', msg.data);

});
```

Esta combinación es muy poderosa para crear interfaces de operador accesibles desde cualquier dispositivo con navegador, sin necesidad de instalar ROS en la máquina del operador.

## Referencias

- [docs.ros.org/en/humble/Tutorials.html](https://docs.ros.org/en/humble/Tutorials.html)
- [design.ros2.org](https://design.ros2.org/)
- [husarion.com/tutorials/ros2-tutorials/ros2](https://husarion.com/tutorials/ros2-tutorials/ros2/)
