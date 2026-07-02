#!/usr/bin/env bash
#
# setup_robotlidar.sh
# ---------------------------------------------------------------------------
#
# Instala: ROS 2 Humble + Gazebo Fortress + stack de navegación (SLAM Toolbox,
# Nav2, IMU tools, robot_localization), clona el repositorio del proyecto,
# resuelve dependencias con rosdep, compila los paquetes de simulación y deja
# el entorno configurado en ~/.bashrc.
#
# USO (dentro de Ubuntu 22.04 / WSL2), desde una carpeta personal:
#   git clone https://github.com/pablem/2025-LidarRobot.git
#   bash 2025-LidarRobot/setup_robotlidar.sh
#
# ---------------------------------------------------------------------------

set -euo pipefail

# ----- Configuración -------------------------------------------------------
ROS_DISTRO="humble"
WORKSPACE="${HOME}/robotLidar"
REPO_URL="https://github.com/pablem/2025-LidarRobot.git"
BASHRC="${HOME}/.bashrc"

# ----- Utilidades de log ---------------------------------------------------
c_green="\033[1;32m"; c_yellow="\033[1;33m"; c_red="\033[1;31m"; c_reset="\033[0m"
log()  { echo -e "${c_green}[✔]${c_reset} $*"; }
step() { echo -e "\n${c_yellow}==> $*${c_reset}"; }
err()  { echo -e "${c_red}[x]${c_reset} $*" >&2; }

# Agrega una línea a ~/.bashrc solo si aún no está (evita duplicados).
append_bashrc_once() {
    local line="$1"
    grep -qxF "$line" "$BASHRC" 2>/dev/null || echo "$line" >> "$BASHRC"
}

# ----- Comprobaciones previas ----------------------------------------------
step "Comprobaciones previas"

if [[ "$(id -u)" -eq 0 ]]; then
    err "No ejecutes este script como root. Usá tu usuario normal (usará sudo cuando haga falta)."
    exit 1
fi

if [[ ! -f /etc/os-release ]]; then
    err "No se detecta /etc/os-release. Este script requiere Ubuntu 22.04."
    exit 1
fi
# shellcheck disable=SC1091
. /etc/os-release
if [[ "${VERSION_ID:-}" != "22.04" ]]; then
    err "Se detectó Ubuntu ${VERSION_ID:-desconocido}. Este script está pensado para 22.04 (Jammy)."
    read -r -p "¿Continuar de todos modos? [s/N] " ans
    [[ "${ans,,}" == "s" ]] || exit 1
fi

# Detecta si estamos en WSL (solo informativo)
if grep -qi microsoft /proc/version 2>/dev/null; then
    log "Entorno WSL2 detectado."
    IS_WSL=1
else
    log "Entorno Linux nativo detectado."
    IS_WSL=0
fi

sudo -v   # solicita la contraseña de sudo una vez al inicio

# ----- 1. Locale -----------------------------------------------------------
step "1/8 Configurando locale (en_US.UTF-8)"
sudo apt update
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
log "Locale configurado."

# ----- 2. Repositorio ROS 2 ------------------------------------------------
step "2/8 Añadiendo el repositorio de ROS 2 Humble"
sudo apt install -y software-properties-common
sudo add-apt-repository universe -y
sudo apt update
sudo apt install -y curl

# Descarga la última versión del paquete que configura el repositorio apt de ROS
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
    | grep -F "tag_name" | awk -F'"' '{print $4}')
UBU_CODENAME=$(. /etc/os-release && echo "${UBUNTU_CODENAME:-${VERSION_CODENAME}}")
curl -L -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${UBU_CODENAME}_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
log "Repositorio ROS 2 configurado (versión apt-source ${ROS_APT_SOURCE_VERSION})."

# ----- 3. ROS 2 Humble + herramientas de desarrollo ------------------------
step "3/8 Instalando ROS 2 Humble Desktop y herramientas de desarrollo"
sudo apt update
sudo apt install -y ros-humble-desktop ros-dev-tools
append_bashrc_once "source /opt/ros/${ROS_DISTRO}/setup.bash"
log "ROS 2 Humble instalado."

# ----- 4. Gazebo Fortress + render por software ----------------------------
step "4/8 Instalando Gazebo Fortress (ros-gz) y configurando render por software"
sudo apt install -y ros-humble-ros-gz
# WSL2 sin GPU: forzar render por software para que Gazebo/RViz funcionen con llvmpipe
append_bashrc_once "export LIBGL_ALWAYS_SOFTWARE=1"
log "Gazebo Fortress instalado. LIBGL_ALWAYS_SOFTWARE=1 añadido a ~/.bashrc."

# ----- 5. Stack de navegación + IMU ----------------------------------------
step "5/8 Instalando SLAM Toolbox, Nav2, imu-tools y robot_localization"
sudo apt install -y \
    ros-humble-slam-toolbox \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-imu-tools \
    ros-humble-robot-localization
log "Stack de navegación e IMU instalado."

# ----- 6. Clonar el proyecto -----------------------------------------------
step "6/8 Clonando el repositorio del proyecto en ${WORKSPACE}/src"
mkdir -p "${WORKSPACE}"
if [[ -d "${WORKSPACE}/src/.git" ]]; then
    log "El repositorio ya existe en ${WORKSPACE}/src, actualizando (git pull)."
    git -C "${WORKSPACE}/src" pull --ff-only || err "No se pudo hacer git pull; se conserva la copia existente."
else
    git clone "${REPO_URL}" "${WORKSPACE}/src"
    log "Repositorio clonado en ${WORKSPACE}/src."
fi

# ----- 7. Dependencias con rosdep ------------------------------------------
step "7/8 Resolviendo dependencias con rosdep"
cd "${WORKSPACE}"
# rosdep init falla si ya fue inicializado: lo hacemos tolerante
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    sudo rosdep init
else
    log "rosdep ya estaba inicializado, se omite 'rosdep init'."
fi
rosdep update
# 'source' de ROS para que rosdep resuelva contra Humble.
# Los scripts de setup de ROS referencian variables sin inicializar
# (p. ej. AMENT_TRACE_SETUP_FILES), así que desactivamos 'nounset'
# temporalmente para que no aborte bajo 'set -u'.
set +u
# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u
rosdep install --from-paths src --ignore-src -r -y
log "Dependencias resueltas."

# ----- 8. Compilación ------------------------------------------------------
step "8/8 Compilando paquetes de simulación con colcon"
cd "${WORKSPACE}"
# Aseguramos el entorno de ROS (protegido contra 'set -u', ver paso 7).
set +u
# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u
colcon build --symlink-install \
    --packages-select robot explore_lite explore_lite_msgs
append_bashrc_once "source ${WORKSPACE}/install/setup.bash"
log "Compilación finalizada."
EOF
