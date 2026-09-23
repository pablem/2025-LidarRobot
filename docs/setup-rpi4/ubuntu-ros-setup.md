---
title: "Ubuntu y ROS: Setup y Configuración"
description: "Ubuntu 22.04 + ROS 2 Humble: PC con desktop, RPi4 con Ubuntu Server y ros-base. Acceso SSH robot_lidar@pi.local, Netplan con DHCP multi-red, Cyclone DDS en todos los equipos. Instalación en un paso con setup_robotlidar.sh; workspace ~/robotLidar."
sidebar:
  order: 1
---

> [!NOTE]
> Este documento cubre dos escenarios de instalación: una **PC de desarrollo** con Ubuntu 22.04 Desktop, y una **Raspberry Pi 4** como computadora embebida del robot con Ubuntu Server 22.04. Las secciones exclusivas de cada plataforma están marcadas con las etiquetas **`[PC]`** y **`[RPi4]`**. Las secciones sin etiqueta aplican a ambas.

| Parámetro | Valor |
| --- | --- |
| Sistema | Ubuntu 22.04 + ROS 2 Humble |
| PC | Ubuntu Desktop |
| RPi4 | Ubuntu Server + `ros-base` |
| Acceso | SSH `robot_lidar@pi.local` |
| Red | Netplan con DHCP multi-red |
| Middleware | Cyclone DDS en todos los equipos |
| Instalación | En un paso con `setup_robotlidar.sh` |
| Workspace | `~/robotLidar` |

## 1. Sistema operativo

### 1.1. Ubuntu 22.04 LTS `[PC]`

```yaml
Datos de mi PC:
	SO Win 10 Pro (22H2)
	Notebook HP 250 G5 (Arq x64)
	Procesador Intel Core i5-6200U (2 núcleos, max freq 2.80 GHz)
	Modo BIOS UEFI (se adaptó anteriormente)
```

Ubuntu 22.04 LTS (Jammy Jellyfish) es la versión de Ubuntu requerida para ROS 2 Humble. La sigla LTS significa Long Term Support.

La imagen de instalación oficial se descarga desde [ubuntu.com/download/desktop](https://ubuntu.com/download/desktop). Para crear un USB booteable se puede usar [Rufus](https://rufus.ie/) en Windows o `dd` en Linux. El proceso de instalación es estándar: arrancar desde el USB, seleccionar idioma, zona horaria, usuario y contraseña, y dejar que el instalador complete el proceso. No se requiere ninguna configuración especial para usar ROS 2.

Hasta ahora una alternativa recomendable es: 

[Simulaciones en Windows con WSL2](../simulacion/windows-wsl2.md)

> [!NOTE]
> **Nota sobre virtualización:** ROS 2 puede usarse dentro de una máquina virtual (VirtualBox, VMware, Hyper-V). Asegurarse de asignar al menos 4 GB de RAM y 30 GB de disco, y habilitar la aceleración de hardware virtual (VT-x/AMD-V) en la BIOS del host. Las herramientas gráficas como rviz2 pueden funcionar con rendimiento reducido en VM sin aceleración 3D.

> [!NOTE]
> **Nota sobre doble booteo (Windows + Ubuntu):** si se instala Ubuntu en dual-boot con Windows asignar correctamente **disco** (particiones) para Ubuntu: como mínimo ~30 GB. **Importante:** desactivar “Inicio rápido”/hibernación de Windows (Fast Startup), porque puede dejar el disco en estado híbrido y causar problemas al montar la partición de Windows desde Ubuntu.
En la práctica, se fuerza (-f) el cierre de todas las aplicaciones, apagando windows con: `shutdown -s -f -t 0`

### 1.2. Ubuntu Server 22.04 LTS `[RPi4]`

Guía: [Install Ubuntu on RPi 4](https://ubuntu.com/tutorials/how-to-install-ubuntu-desktop-on-raspberry-pi-4#1-overview)

La Raspberry Pi 4 no tiene un instalador gráfico convencional: el sistema operativo se escribe directamente en una tarjeta microSD usando la herramienta oficial **Raspberry Pi Imager** ([raspberrypi.com/software](https://www.raspberrypi.com/software/)). Se instala en la PC, se selecciona la imagen y la tarjeta destino, y el Imager graba todo automáticamente.

La combinación recomendada para usar con ROS 2 Humble es **Ubuntu Server 22.04.5 LTS (64-bit)**. A diferencia de Ubuntu Desktop, la variante Server no tiene entorno gráfico: todo se opera mediante línea de comandos, ya sea directamente o por SSH. Esto es porque nuestro robot no tendrá pantalla ni teclado en una operación normal.

Antes de grabar la imagen, el Imager permite preconfigurar el sistema mediante los ajustes avanzados (ícono de engranaje). Conviene configurar todo en este paso para poder acceder a la RPi4 directamente por SSH sin necesidad de conectar monitor ni teclado:

- **Hostname:** nombre con el que la RPi4 aparecerá en la red (ej. `pi`). Permite conectarse como `ssh usuario@pi.local` sin necesitar conocer la IP.
- **Usuario y contraseña:** las credenciales del usuario principal del sistema.
- **Wi-Fi:** SSID y contraseña de la red. El Imager configura la conexión automáticamente al primer arranque.
- **SSH:** habilitar con autenticación por contraseña.

> [!NOTE]
> **Sobre la tarjeta SD:** la Raspberry Pi 4 no tiene disco interno; el sistema operativo vive en la microSD, por lo que su rendimiento impacta directamente en los tiempos de arranque y en la velocidad general del sistema. Para Ubuntu Server + ROS 2 lo crítico no es la velocidad secuencial sino los **IOPS** (clase **A2**, ≥4000 IOPS de lectura).

---

## 2. Acceso y configuración de red `[RPi4]`

### 2.1. Acceso por SSH `[RPi4]`

Una vez que la RPi4 arranca con la imagen configurada, es accesible por SSH desde cualquier máquina en la misma red. Si se configuró el hostname en el Imager, no es necesario conocer la IP:

```bash
ssh robot_lidar@pi.local
# luego ingresar la contraseña configurada en el Imager
```

Si el hostname no resuelve, se puede obtener la IP directamente en la RPi4 (con monitor y teclado conectados temporalmente), o desde otro dispositivo en la red:

```bash
# En la RPi4:
hostname -I

# Desde otra PC Linux con conexión previa:
ip neigh
```

Para transferir archivos entre la PC y la RPi4 sin necesidad de extraer la SD, se usa `scp` (Secure Copy), que funciona sobre el mismo canal SSH:

```bash
scp /ruta/archivo/local robot_lidar@pi.local:/ruta/destino/
```

### 2.2. Configuración de red con Netplan `[RPi4]`

La configuración de red en Ubuntu Server se gestiona con **Netplan**, que usa archivos YAML en `/etc/netplan/`. Por defecto, Ubuntu Server crea un archivo `50-cloud-init.yaml` gestionado automáticamente por cloud-init. Para tener control total sobre la red, se desactiva cloud-init para red y se crea un archivo propio.

Primero, deshabilitar la gestión de red por cloud-init:

```bash
sudo nano /etc/cloud/cloud.cfg.d/99-disable-network-config.cfg
```

Agregar el siguiente contenido y guardar:

```
network: {config: disabled}
```

Luego renombrar el archivo original para que Netplan no lo tome:

```bash
sudo mv /etc/netplan/50-cloud-init.yaml /etc/netplan/50-cloud-init.yaml.bak
```

A continuación, crear el archivo de configuración propio. Hay dos estrategias según el caso de uso:

**Opción A — IP fija:** la RPi4 siempre tendrá la misma IP, lo que simplifica la conexión SSH. Requiere saber de antemano la IP del gateway y el rango disponible en la red:

```bash
sudo nano /etc/netplan/51-config.yaml
```

```yaml
network:
  version: 2
  renderer: networkd

  wifis:
    wlan0:
      dhcp4: no
      addresses: [192.168.100.55/24]
      gateway4: 192.168.100.1
      nameservers:
        addresses: [192.168.100.1]
      access-points:
        NombreDeLaRed:
          password: "contraseña"

  ethernets:
    eth0:
      dhcp4: true   # IP automática si se conecta por cable
```

**Opción B — DHCP con múltiples redes:** útil cuando el robot necesita conectarse a distintas redes. La desventaja es que la IP puede cambiar entre sesiones. Para resolverlo sin conocer la IP, la mejor opción es usar el hostname (`ssh robot_lidar@pi.local`). Es la estrategia adoptada, porque el robot se traslada entre el laboratorio y otros entornos de prueba.

> [!NOTE]
> **Otra alternativa:** la app IP Tools en el celular para escanear la red local, o la lista de dispositivos del router para buscar la entrada con nombre `pi`

> [!NOTE]
> **Otra alternativa 2:** Zona WiFi (Hotspot) a partir de Android 10 tiene la capacidad de visualizar los dispositivos conectados, junto con sus direcciones IP y MAC.

```yaml
network:
  version: 2
  renderer: networkd

  wifis:
    wlan0:
      optional: true
      dhcp4: true
      access-points:
        Red1:
          password: "contraseña1"
        Red2:
          password: "contraseña2"

  ethernets:
    eth0:
      dhcp4: true
```

Para aplicar los cambios:

```bash
sudo netplan try      # aplica cambios por 60 s y revierte automáticamente si hay error de sintaxis
sudo netplan apply    # aplica cambios de forma permanente
```

---

## 3. Preparación del sistema

### 3.1. Actualizar el sistema

```bash
sudo apt update && sudo apt upgrade -y
```

Este paso descarga e instala todas las actualizaciones disponibles para el sistema base. Es importante hacerlo antes de instalar ROS 2 para evitar conflictos de versiones entre paquetes.

### 3.2. Configurar el locale `[PC]`

ROS 2 requiere que el sistema esté configurado con un locale que soporte UTF-8. En una instalación limpia de Ubuntu 22.04 Desktop esto generalmente ya está configurado, pero conviene verificarlo y forzarlo explícitamente:

```bash
sudo apt install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

Para verificar:

```bash
locale
# La salida debe mostrar LANG=en_US.UTF-8
```

### 3.3. Habilitar el repositorio Universe de Ubuntu

El repositorio Universe contiene software de código abierto mantenido por la comunidad. Algunos paquetes de dependencias de ROS 2 se encuentran ahí:

```bash
sudo apt install software-properties-common -y
sudo add-apt-repository universe
```

---

## 4. Instalación de ROS 2 Humble

El método elegido es mediante el paquete **`ros2-apt-source`**, que configura el repositorio de ROS 2 de forma que las actualizaciones del propio repositorio se gestionan automáticamente con `apt`, sin necesidad de intervención manual cuando se lanzan nuevas versiones.

### 4.1. Agregar el repositorio de ROS 2

Instalación vía Paquete .deb (ros-apt-source)

```bash
sudo apt update && sudo apt install curl -y

export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')

curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"

sudo dpkg -i /tmp/ros2-apt-source.deb
```

### 4.2. Instalar ROS 2

Con el repositorio configurado, actualizar el índice de paquetes e instalar. La variante difiere según la plataforma:

```bash
sudo apt update && sudo apt upgrade -y
```

**En PC `[PC]`** — instalar `ros-humble-desktop`, que incluye las bibliotecas de ROS 2, herramientas de línea de comandos, ejemplos, rviz2 y rqt:

```bash
sudo apt install ros-humble-desktop -y
```

**En Raspberry Pi 4 `[RPi4]`** — instalar `ros-humble-ros-base`, que incluye únicamente las bibliotecas de comunicación, paquetes de mensajes y herramientas de línea de comandos, sin herramientas gráficas. Esto reduce significativamente el uso de disco y RAM, lo cual es relevante en hardware embebido:

```bash
sudo apt install ros-humble-ros-base -y
```

### 4.3. Instalar herramientas de desarrollo

En ambas plataformas, instalar el paquete de herramientas de desarrollo, que incluye `colcon`, `rosdep` y otras utilidades necesarias para compilar paquetes:

```bash
sudo apt install ros-dev-tools -y
```

### 4.4. Dependencias adicionales del proyecto

Además del paquete base, el proyecto requiere los siguientes paquetes de ROS 2:

```bash
sudo apt install \
  ros-humble-xacro ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher ros-humble-ros2-control \
  ros-humble-ros2-controllers ros-humble-robot-localization \
  ros-humble-slam-toolbox ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-twist-mux ros-humble-tf2-ros ros-humble-tf2-tools \
  ros-humble-rqt-tf-tree -y
```

> [!TIP]
> **Instalación en un paso:** el repositorio incluye un script que instala, configura y compila todo el proyecto:

```bash
git clone https://github.com/pablem/2025-LidarRobot.git
bash 2025-LidarRobot/setup_robotlidar.sh
```

---

## 5. Configuración del entorno

### 5.1. Source del setup script

Para poder usar los comandos `ros2` en una terminal, es necesario inicializar el entorno ejecutando el script de configuración de ROS 2. Este script configura las variables de entorno necesarias (`PATH`, `PYTHONPATH`, `AMENT_PREFIX_PATH`, etc.):

```bash
source /opt/ros/humble/setup.bash
```

Este comando debe ejecutarse en cada terminal nueva que se quiera usar con ROS 2. Para automatizarlo, se agrega al archivo de inicialización del shell:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 5.2. Verificar la instalación

```bash
ros2 doctor --report | grep ros2cli
	# ros2cli_common_extensions
	# ros2cli 
```

En la PC, donde se instaló `desktop`, se puede hacer una verificación más completa con el ejemplo de publisher/subscriber incluido. En una terminal:

```bash
ros2 run demo_nodes_cpp talker
```

Y en otra terminal:

```bash
ros2 run demo_nodes_py listener
```

Si la instalación es correcta, el `talker` publicará mensajes `Hello World: N` y el `listener` los recibirá e imprimirá en pantalla. Ambos se detienen con `Ctrl+C`.

---

## 6. rosdep

`rosdep` es una herramienta de ROS que resuelve e instala automáticamente las dependencias del sistema que requieren los paquetes ROS. Es especialmente útil al clonar paquetes de terceros. Antes de usarlo por primera vez, hay que inicializarlo:

```bash
sudo rosdep init
rosdep update
```

El primero crea el archivo de configuración en `/etc/ros/rosdep/` (requiere sudo). El segundo descarga el índice de dependencias conocidas desde los repositorios de ROS. Para instalar todas las dependencias de los paquetes en el workspace actual:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

---

## 7. Workspace y paquetes

### 7.1. Crear el workspace

Por convención, el workspace del proyecto se crea en el directorio home del usuario. El nombre puede ser cualquiera; en este proyecto se usa `robotLidar`:

```bash
mkdir -p ~/robotLidar/src
cd ~/robotLidar
colcon build --symlink-install
```

El flag `--symlink-install` crea enlaces simbólicos en lugar de copiar archivos, lo que permite que los cambios en scripts Python y archivos de configuración se reflejen sin necesidad de recompilar.

Una vez compilado el workspace, agregar su source al `.bashrc`. El source del workspace debe ir siempre **después** del source de ROS 2:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/robotLidar/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

A partir de aquí, todos los paquetes del proyecto se colocan dentro de `src/` y se compilan con `colcon build` desde la raíz del workspace.

Paquetes del workspace:

| Paquete | Origen | Contenido |
| --- | --- | --- |
| `robot` | Propio | Launch, URDF/xacro, config, mundos de simulación y scripts |
| `diffdrive_arduino` | Terceros (modificado) | Hardware interface de ros2_control; parámetros por rueda y lectura de batería |
| `serial` | Terceros | Biblioteca C++ de comunicación serie |
| `lidar_driver` | Adaptado | Driver del XV-11 portado de ROS 1 a Humble |
| `mpu9250driver` | Adaptado | Driver I²C de la IMU; unidades SI y corrección del magnetómetro |
| `m-explore-ros2` | Terceros | explore_lite (variante compatible con slam_toolbox) |

### 7.2. Crear un paquete con CMake

Los paquetes C++ de ROS 2 usan el sistema de build `ament_cmake`. Para crear un paquete nuevo dentro del workspace:

```bash
cd ~/robotLidar/src
ros2 pkg create --build-type ament_cmake robot
```

> [!NOTE]
> Modificar el archivo `CMakeLists.txt` para indicar a colcon cómo instalar los directorios de recursos:
>
>
> ```
> install(
>   DIRECTORY config description launch worlds
>   DESTINATION share/${PROJECT_NAME}
> )
> ```

> [!NOTE]
> **POR REVISAR:** Agregar dependencias en `package.xml`. Este archivo indica a ROS y a colcon información adicional sobre el paquete, incluyendo sus dependencias en tiempo de compilación y ejecución:
>
>
> ```xml
> <exec_depend>demo_nodes_cpp</exec_depend>
> ```

---

## 8. CLI útiles de Linux

### 8.1. Dispositivos USB y puertos serie

Para identificar un dispositivo USB por su enlace simbólico único basado en fabricante, modelo y número de serie, lo cual es más confiable que usar `/dev/ttyUSB0` o `/dev/ttyACM0` ya que estos pueden cambiar según el orden de conexión:

```bash
ls -l /dev/serial/by-id/
# Ejemplo de salida:
# usb-Espressif_USB_JTAG_serial_debug_unit_10:20:BA:4D:92:80-if00 -> ../../ttyACM0   (ESP32-S3, motores)
# usb-Arduino_LLC_Arduino_Leonardo-if00 -> ../../ttyACM1                            (Pro Micro, LiDAR)

# Usar el id en un nodo ROS:
ros2 run serial_motor_demo driver --ros-args \
  -p serial_port:=/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_10:20:BA:4D:92:80-if00

# Listar puertos serie disponibles:
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

### 8.2. Comunicación serie con screen `[RPi4]`

Para interactuar directamente con un dispositivo conectado de forma dinámica por puerto serie y poder logear la salida se utilizó `screen` 

```bash
# instalar herramienta:
sudo apt install screen

# guardar datos en la carpeta actual:
rm test_pwm.txt # evitar apend
screen -L -Logfile test_pwm.txt /dev/ttyACM0 57600

# ejemplo: cambiar las ganancias PI de ambos motores (Kp_izq:Ki_izq:Kp_der:Ki_der)
screen /dev/ttyACM0 57600
u 0.0807:1.131:0.0898:1.224
# "OK"
```

Para salir:

`Ctrl + A`
`Luego: K`
`Confirmar con Y`

Para desactivar el log:

`Ctrl + A`
`H`

---

## 9. Optimización de boot `[RPi4]`

En la Raspberry Pi 4 con Ubuntu Server y ROS 2, el tiempo de arranque inicial puede ser elevado (del orden de 5 minutos en algunos casos). Las acciones de mejora se dividen en dos planos:

- **Hardware:** la microSD es el principal cuello de botella. Antes (o en paralelo) a las optimizaciones de software, evaluar una tarjeta con certificación **A2** o un SSD por USB 3.0. Ver ‣.
- **Software:** desactivar servicios y esperas innecesarias (lo que sigue en esta sección).

Para diagnosticar qué servicios demoran más:

```bash
systemd-analyze blame
systemctl list-unit-files --state=enabled
```

**Cambiar el target de arranque:** por defecto Ubuntu Server puede tener configurado `graphical.target` aunque no tenga entorno gráfico instalado. Cambiarlo a `multi-user.target` elimina esa espera:

```bash
sudo systemctl set-default multi-user.target
# Revertir: sudo systemctl set-default graphical.target
```

**Deshabilitar la espera de red en el arranque:** el servicio `systemd-networkd-wait-online` hace que el sistema espere hasta que la red esté completamente disponible antes de continuar, lo que puede agregar hasta 2 minutos. Al deshabilitarlo, el arranque continúa inmediatamente y la red se levanta en paralelo:

```bash
sudo systemctl disable systemd-networkd-wait-online.service
```

**Eliminar cloud-init:** sistema diseñado para configuración inicial de máquinas virtuales en la nube. No tiene utilidad en una RPi4 y agrega tiempo de arranque:

```bash
sudo systemctl disable cloud-init cloud-config cloud-final cloud-init-local
sudo apt purge cloud-init -y
```

**Eliminar snap:** gestor de paquetes alternativo a apt que no se usa en este proyecto:

```bash
sudo apt purge snapd -y
sudo rm -rf ~/snap /snap /var/snap
```

**Deshabilitar Bluetooth** si no se usa:

```bash
sudo systemctl disable bluetooth hciuart
```

Luego de aplicar estos cambios, reiniciar y verificar la mejora:

```bash
systemd-analyze
# Ejemplo medido tras las optimizaciones:
# Startup finished in 10.631s (kernel) + 2min 9.604s (userspace) = 2min 20.236s
# multi-user.target reached after 2min 9.555s in userspace
```

Si alguno de los tiempos sigue siendo alto, repetir `systemd-analyze blame` para identificar el siguiente servicio a deshabilitar o auditar.

### 9.1. Ajuste de gobernador de CPU

El gobernador de CPU controla cómo el procesador ajusta su frecuencia de funcionamiento. El valor por defecto en la RPi4 es `ondemand`, que sube la frecuencia al máximo cuando hay carga y la baja cuando no la hay, equilibrando rendimiento y consumo.

Para ver el gobernador activo y los disponibles:

```bash
cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_available_governors
```

Para cambiar temporalmente a máximo rendimiento:

```bash
sudo cpufreq-set -g performance
# Volver a ondemand:
sudo cpufreq-set -g ondemand
```

Si el kernel lo soporta, `schedutil` es una alternativa moderna integrada con el planificador del kernel, recomendada en kernels recientes. Para monitorear las frecuencias reales de todos los núcleos en tiempo real:

```bash
watch -n1 "cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_cur_freq"
```

---

## 10. Middleware DDS (Cyclone DDS)

ROS 2 usa una capa intermedia llamada **RMW** (ROS Middleware) que abstrae la implementación DDS subyacente. DDS (Data Distribution Service) es el motor de comunicación publicación-suscripción que conecta nodos, posiblemente distribuidos en distintas máquinas, con baja latencia y descubrimiento automático.

ROS 2 Humble trae **Fast DDS** por defecto. **Cyclone DDS** es una implementación alternativa de código abierto, conocida por su baja latencia y mejor consistencia temporal en sistemas distribuidos (por ejemplo, una PC + una RPi4 publicando datos de sensores a alta frecuencia por Wi-Fi).

### 10.1. Verificar la implementación activa

```bash
ros2 doctor --report
# Buscar la sección RMW MIDDLEWARE
# middleware name : rmw_fastrtps_cpp   (Fast DDS, valor por defecto)
```

### 10.2. Instalar Cyclone DDS

Instalar en **todos** los equipos que vayan a comunicarse entre sí (PC y RPi4):

```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp
```

### 10.3. Activar Cyclone DDS

Activación temporal (solo afecta a la terminal actual):

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

Activación permanente para el usuario:

```bash
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
source ~/.bashrc
```

Para revertir a Fast DDS, eliminar la línea del `~/.bashrc` (o exportar otra implementación).

> [!NOTE]
> **Importante:** todos los nodos que se quieran comunicar entre sí deben usar la misma implementación RMW. Si la PC usa Cyclone DDS y la RPi4 sigue con Fast DDS, los nodos no se descubrirán entre sí.

## Referencias

- [ubuntu.com/tutorials/how-to-install-ubuntu-desktop-on-raspberry-pi-4#1-overview](https://ubuntu.com/tutorials/how-to-install-ubuntu-desktop-on-raspberry-pi-4#1-overview)
- [ubuntu.com/download/desktop](https://ubuntu.com/download/desktop)
- [rufus.ie](https://rufus.ie/)
- [raspberrypi.com/software](https://www.raspberrypi.com/software/)
- [github.com/ros-infrastructure/ros-apt-source](https://github.com/ros-infrastructure/ros-apt-source)
- [github.com/pablem/2025-LidarRobot](https://github.com/pablem/2025-LidarRobot)
