---
title: "Diccionario"
sidebar:
  order: 3
---

## PSRAM

PSRAM, o memoria pseudoestática de acceso aleatorio, es una memoria dinámica (DRAM) con un circuito de refresco integrado que la hace comportarse externamente como una memoria estática (SRAM). Esto permite combinar las ventajas de la alta densidad de la DRAM con la simplicidad de acceso de la SRAM, siendo más densa y económica que la SRAM pura, y más fácil de usar que la DRAM estándar. Se usa comúnmente en aplicaciones de microcontroladores (como el ESP32) para aumentar la memoria disponible para tareas como el almacenamiento en búfer de datos de red o para almacenar imágenes temporales de una cámara.

## URDF

URDF (Unified Robot Description Format) es el archivo XML donde se describe la geometría completa del robot: qué links (piezas) tiene, qué joints (uniones) los conectan, y cuál es la relación espacial entre ellos. Cada joint en el URDF define una transformada estática o dinámica entre un frame padre y un frame hijo. El nodo robot_state_publisher lee el URDF y publica todos esos transforms en el topico /tf_static, que es de donde TF2 los consume.

## XACRO

Xacro (XML Macro) es una extensión del lenguaje XML que se usa URDF que permite usar macros, variables y includes para que el archivo sea modular, escalable, etc. 

## TF2

TF2 es el sistema que mantiene la relación espacial entre los distintos marcos de referencia o FRAMES del robot. Cada frame representa un sistema de coordenadas anclado a alguna parte física o lógica del robot. TF sabe cómo transformar una medición expresada en un frame a cualquier otro frame del árbol. Esto es fundamental porque los sensores miden en su propio sistema de coordenadas local. El lidar "ve" obstáculos relativos a laser_frame. La IMU mide aceleración relativa a imu_frame. Para que Nav2, SLAM Toolbox y el EKF puedan combinar esa información, necesitan saber exactamente dónde está cada sensor respecto al cuerpo del robot.

## REP

ROS Enhancement Proposal: es un documento técnico que define convenciones, estándares y buenas prácticas dentro del ecosistema ROS. Si bien es específico de la comunidad ROS, es comparable con normalización de organismos internacionales/nacionales. 

## Loop Closure

El cierre de bucle consiste en reconocer una ubicación visitada previamente y corregir los errores acumulativos en el mapa.

## Pose grafo (grafo de poses)

Es una estructura de datos matemática utilizada para mapear y localizar. Representa la trayectoria del robot como nodos (posiciones y orientaciones) conectados por aristas (movimientos medidos), optimizando todo el mapa para corregir errores de odometría al detectar cierres de bucle.

## Pose

Representación de la ubicación espacial (x,y,z) y orientación (ángulo/rotación) en un sistema de dimensiones. 

## Odometría

Estimación de la pose del robot a partir de la integración de su propio movimiento (en este robot, las cuentas de los encoders de cada rueda). Es continua y suave, ideal para el control, pero acumula error con el tiempo porque cualquier deslizamiento o imprecisión en las dimensiones se suma paso a paso. Por eso se corrige con otras fuentes: la IMU (a través del EKF) y el LiDAR (a través de SLAM).

## Twist

Mensaje estándar de ROS (`geometry_msgs/Twist`) que expresa una velocidad mediante dos vectores: lineal (vx, vy, vz) y angular (ωx, ωy, ωz). En un robot diferencial sólo tienen sentido `linear.x` (avanzar o retroceder) y `angular.z` (girar sobre su eje); el resto se deja en cero.

## EKF (Filtro de Kalman Extendido)

Algoritmo de estimación que combina varias mediciones ruidosas para obtener una estimación más confiable que la de cualquier sensor por separado. Funciona en dos etapas: predice el estado con un modelo del movimiento y lo corrige con las mediciones, ponderando cada una según su incertidumbre. Es la versión para sistemas no lineales del filtro de Kalman. En el proyecto, el paquete robot_localization lo usa para fusionar la odometría de los encoders con el giróscopo de la IMU.

## Covarianza

Medida de la incertidumbre de una variable. En ROS, cada mensaje de sensor puede incluir una matriz de covarianza cuya diagonal contiene las varianzas (σ²) de cada magnitud: un valor chico indica una medición confiable y uno grande, una medición ruidosa. Filtros como el EKF usan estos valores para decidir cuánto confiar en cada fuente.

## Costmap (mapa de costos)

Grilla de ocupación que usa Nav2 para planificar, donde cada celda guarda un costo de tránsito entre 0 (libre) y 254 (obstáculo). Se construye por capas: el mapa estático de SLAM, los obstáculos que detecta el LiDAR en vivo y una zona de inflado alrededor de cada obstáculo para que el robot no pase rozando. Hay uno global (todo el entorno, para la ruta completa) y uno local (una ventana alrededor del robot, para esquivar en tiempo real).

## Frontera

En exploración autónoma, es el borde entre el espacio ya conocido y libre del mapa y el espacio todavía desconocido. Un explorador por fronteras (como explore_lite) elige la frontera más conveniente, envía al robot hacia ella y repite el proceso hasta que no quedan fronteras alcanzables.

## Cuaternión

Forma de representar una orientación en 3D mediante cuatro números q = [w, x, y, z]: la parte vectorial (x, y, z) indica el eje de rotación y la escalar (w) se relaciona con el ángulo girado. ROS lo usa en lugar de los ángulos roll/pitch/yaw porque evita ambigüedades y singularidades (como el bloqueo de cardan) y es más eficiente para componer rotaciones.

## DDS / RMW

DDS (Data Distribution Service) es el estándar de comunicación publicación-suscripción sobre el que corre ROS 2: se encarga de que los nodos se descubran solos en la red y de transportar los mensajes. RMW (ROS Middleware) es la capa que permite elegir qué implementación de DDS usar (por ejemplo Fast DDS, que viene por defecto, o Cyclone DDS, la que usa este proyecto). Todos los equipos que se comunican deben usar la misma.

## PCNT (Pulse Counter)

Periférico de hardware del ESP32 que cuenta flancos en pines de entrada sin intervención de la CPU. Se usa para leer encoders en cuadratura: el hardware mantiene el conteo y el programa sólo lo consulta cuando lo necesita, sin interrupciones por cada pulso. El ESP32-S3 tiene dos unidades PCNT, justo una por encoder.

## MCPWM

Motor Control PWM: periférico del ESP32 diseñado para accionar motores. A diferencia del PWM de uso general (LEDC), sus temporizadores tienen salidas en pares pensadas para comandar las dos entradas de un puente H, lo que simplifica invertir el sentido de giro.

## OTA (Over-The-Air)

Carga del firmware por la red inalámbrica, sin cable USB. En el proyecto se usa ArduinoOTA sobre WiFi para reprogramar el ESP32-S3 con el robot en movimiento libre, lo que fue clave para los ensayos de identificación de los motores.

## Zona muerta

Rango de la señal de control en el que el actuador no responde. En un motor DC es el ciclo de trabajo mínimo por debajo del cual no vence la fricción y no arranca (en este robot, del orden de 1,6 V). Afecta al lazo de control porque el término integral debe acumularse hasta superar ese umbral antes de que la rueda gire.

## Anti-windup

Mecanismo que evita que el término integral de un controlador siga acumulándose cuando la salida ya está saturada (por ejemplo, PWM al 100 %). Sin él, la integral crece de más y el sistema reacciona tarde y con sobreimpulso al salir de la saturación. El firmware usa integración condicional: sólo integra mientras la salida no está saturada.
