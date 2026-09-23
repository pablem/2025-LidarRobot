---
title: "Linux: Introducción breve"
description: "Conceptos. Línea de comandos más usados."
sidebar:
  order: 1
---

> [!NOTE]
> En este proyecto se utilizarán dos variantes: **Ubuntu Desktop 22.04** en la PC, y **Ubuntu Server 22.04** en la Raspberry Pi 4 (no tiene entorno gráfico, se maneja exclusivamente por consola)

## Recursos

https://www.gnu.org/software/bash/manual/bash.html

https://info-ee.surrey.ac.uk/Teaching/Unix/

https://explainshell.com/

## Todo es un archivo

En sistemas tipo Unix, prácticamente todos los recursos del sistema se representan como archivos dentro de un árbol de directorios. Esto incluye dispositivos, procesos y configuraciones, que se manipulan mediante operaciones estándar de lectura y escritura: discos, puertos seriales, buses I2C (como los disponibles en `/dev/i2c-*` en la Raspberry Pi), USB, cámaras, sensores, entre otros. Por ejemplo, `/dev/ttyUSB0` representa típicamente un microcontrolador conectado por USB.

## Directorios importantes

| Ruta | Descripción |
| --- | --- |
| `/home` | Carpetas de usuarios |
| `/etc` | Configuración del sistema |
| `/dev` | Dispositivos conectados (discos, seriales, I2C, USB, etc.) |
| `/usr` | Programas instalados |
| `/var` | Logs y datos variables |
| `/tmp` | Archivos temporales |

## CLI básica

```bash
# ubicación actual
pwd

# listar archivos
ls
ls -l
ls -la

# cambiar directorio
cd carpeta
cd ..
cd ~

# crear
mkdir carpeta
touch archivo.txt

# copiar / mover / eliminar
cp origen destino
mv archivo nuevo_nombre
rm archivo.txt
rm -r carpeta/

# limpiar terminal
clear

# cancelar proceso
CTRL + C

# autocompletado
TAB
```

## Editores de texto en terminal

El editor más accesible para principiantes es **nano**. Se abre con `nano archivo.txt`, se guarda con `CTRL + O` y se cierra con `CTRL + X`.

## SSH: acceso remoto

SSH permite controlar otra computadora desde la terminal, y es la forma habitual de trabajar con la Raspberry Pi. Para conectarse: `ssh usuario@ip`, por ejemplo `ssh ubuntu@192.168.1.50`.

## Red

Cada dispositivo en la red se identifica por una dirección IP (ej. `192.168.1.50`). La dirección `127.0.0.1` (también llamada `localhost`) siempre refiere a la propia máquina.

## Procesos y monitoreo

Para ver los procesos activos se usa `top`, aunque `htop` resulta más cómodo (se instala con `sudo apt install htop`). Los logs del sistema se consultan con `journalctl`, y en tiempo real con `journalctl -f` (Ubuntu usa `systemd` como gestor de servicios).

## Permisos

Cada archivo tiene permisos definidos para su propietario, su grupo y el resto de usuarios. La cadena `-rwxr-xr--` se lee así: `r` es lectura, `w` escritura y `x` ejecución. Se recomienda usar `sudo` sólo cuando sea estrictamente necesario, en lugar de trabajar permanentemente como root.

## Comandos de diagnóstico rápido

```bash
ip a          # ver dirección IP
df -h         # espacio en disco
free -h       # uso de memoria
sudo reboot   # reiniciar
sudo poweroff -h now # apagar ahora
sudo apt update && sudo apt upgrade  # mantener el sistema actualizado
```

## Referencias

- [info-ee.surrey.ac.uk/Teaching/Unix](https://info-ee.surrey.ac.uk/Teaching/Unix/)
- [gnu.org/software/bash/manual/bash.html](https://www.gnu.org/software/bash/manual/bash.html)
- [explainshell.com](https://explainshell.com/)
