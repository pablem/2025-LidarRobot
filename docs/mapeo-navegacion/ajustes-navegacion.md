---
title: "Ajustes y Observaciones durante la Navegación"
description: "Diagnóstico y ajuste fino de Nav2 + SLAM Toolbox en navegación real: costmap, scan matching, latencia del XV-11 y su impacto en la localización."
sidebar:
  order: 4
---

> [!NOTE]
> Registro de problemas observados durante sesiones de navegación autónoma real, su diagnóstico, causa raíz identificada y los ajustes aplicados. Todos los cambios refieren a los archivos `mapper_params_online_async.yaml` y `navegation2_params_waffle_mod.yaml` del repositorio.

## Síntoma principal: colisiones en esquinas después de girar

Durante las sesiones de navegación autónoma con `explore_lite`, se observó que el robot colisionaba con obstáculos con mayor frecuencia al completar un giro. En RViz2, el footprint del robot (polígono del costmap) se desplazaba visiblemente por celdas rojas (zonas de costo máximo) que ya existían al momento de trazar la ruta. En ese momento el modelo del robot era un footprint poligonal rectangular; más adelante se reemplazó por un modelo circular (`robot_radius`), por las razones que se detallan en la sección de modelo del robot más abajo.

Esto ocurría a pesar de que:

- La odometría filtrada por EKF (`odom`) era precisa y coincidía con la posición real del robot.
- Los obstáculos estaban correctamente marcados en el costmap antes de planificar.
- El costmap `global` trabajaba en el frame `map`, donde SLAM Toolbox publica la pose del robot.

---

## Indicios clave

El monitor de TF mostró un delay negativo sistemático:

```bash
ros2 run tf2_ros tf2_monitor map odom
# Average Delay: -0.247323  Max Delay: 0.0331845
```

Y el topic `/scan` presentó un retardo constante de ~300 ms:

```bash
ros2 topic delay /scan
# average delay: 0.301s  std dev: 0.001s
```

---

## Causa raíz: latencia inherente del XV-11

> [!NOTE]
> La latencia inherente de un lidar giratorio lento (XV-11, ~5 Hz) introduce un retardo de timestamp de ~300 ms que es invisible en traslación pero se manifiesta como error angular durante rotaciones, degradando la localización justo en las esquinas. Es un buen ejemplo de cómo una limitación de hardware barato se propaga hasta el comportamiento de navegación, y de por qué la fusión con odometría de encoders (sin latencia) vía EKF es importante.

El XV-11 gira a ~5 Hz (una vuelta cada ~200 ms). El driver captura el timestamp con `t_start = this->now()` **antes** del `poll()` bloqueante, pero el scan completo solo está disponible ~200 ms después. Sumado a la latencia de transporte serie/USB, el retardo total es de ~300 ms con muy baja varianza (σ ≈ 1 ms), lo que confirma que es estructural, no un problema de jitter.

Este retardo es inocuo en traslación: a 0.18 m/s durante 300 ms el error de posición es ~5 cm, asumible. Pero en **rotación** la física cambia:

- A `max_vel_theta = 0.6 rad/s`, un retardo de 300 ms equivale a **~10° de error angular**.
- SLAM Toolbox usa el timestamp del scan para anclar la corrección `map → odom`. Si ese timestamp apunta 300 ms atrás, la corrección se aplica a la pose que el robot tenía entonces, no a la actual.
- Resultado: el footprint en el frame `map` queda rotado respecto a la posición real justo mientras el robot completa el giro en la esquina.

El efecto es sistemático pero solo visible en rotación, lo que explica el patrón observado.

**Por qué el EKF mitiga esto en `odom` pero no en `map`:** la odometría de encoders (sin latencia) que publica `odom → base_footprint` es correcta en todo momento. El error aparece únicamente cuando SLAM Toolbox corrige `map → odom` con un scan desactualizado, desplazando el frame `map` respecto al `odom` preciso.

---

## Ajustes aplicados

### 1. `transform_tolerance` en ambos costmaps

El `transform_tolerance` por defecto (0.3 s) era insuficiente dado el delay sistemático de ~0.25 s observado en la TF `map → odom`. Cuando el delay excede la tolerancia, el costmap descarta o extrapola mal la transformada, produciendo saltos en el footprint.

```yaml
# local_costmap y global_costmap
transform_tolerance: 0.5   # antes: default 0.3
```

### 2. Reducción de `max_vel_theta`

Como el error angular es proporcional a `ω × Δt`, reducir la velocidad angular máxima reduce el error directamente:

```yaml
max_vel_theta: 0.4   # antes: 0.6 rad/s → error ~10°; ahora ~7°
```

### 3. Penalties de scan matching en SLAM Toolbox

Como la odometría filtrada por EKF es más confiable que el scan matcher durante rotaciones, se penaliza más la desviación angular respecto al odom. Valores más bajos significan que al optimizador le "cuesta más" alejarse de la pose que indica la odometría:

```yaml
# mapper_params_online_async.yaml
distance_variance_penalty: 0.3   # antes: 0.5
angle_variance_penalty: 0.4      # antes: 1.0  ← el cambio más importante
```

> [!NOTE]
> **Nota sobre la semántica:** estos son parámetros de varianza, no de peso. Un valor **más alto** permite mayor desviación (confía menos en el odom); un valor **más bajo** penaliza más la desviación (prioriza el odom).

### 4. Frecuencia de procesamiento de scans durante rotación

`minimum_travel_heading: 0.1` rad (≈ 5.7°) hacía que SLAM procesara un scan nuevo cada ~5° de rotación, generando hasta 16 correcciones por cada giro de 90°. Al subir el umbral a ~17°, se reduce a ~5 correcciones por giro:

```yaml
minimum_travel_distance: 0.15   # antes: 0.1 m
minimum_travel_heading: 0.3     # antes: 0.1 rad (~5.7° → ~17°)
```

### 5. `cost_scaling_factor` e `inflation_radius` — gradiente de inflación

Un `cost_scaling_factor` alto produce un gradiente abrupto: el costo cae rápido con la distancia al obstáculo, dando al planner poco incentivo para mantenerse alejado. Un factor bajo extiende el gradiente y hace que el planner prefiera el centro del pasillo.

```yaml
# local_costmap
inflation_layer:
  inflation_radius: 0.30       # antes: 0.30
  cost_scaling_factor: 1.2     # antes: 2.5

# global_costmap
inflation_layer:
  inflation_radius: 0.40       # > local; deja goals/frontiers menos pegados a obstáculos
  cost_scaling_factor: 1.2     # antes: 2.0
```

Dos reglas prácticas que surgieron al pasar al modelo circular (`robot_radius: 0.33`):

- **`inflation_radius` debe ser ≥ `robot_radius`** para que exista una zona de gradiente. El gradiente útil aparece entre el radio del robot y `inflation_radius`; si `inflation_radius ≤ robot_radius`, las celdas pasan de letales a libres sin transición y `cost_scaling_factor` no tiene efecto. El local quedó en 0.30 (apenas por debajo de 0.33) a propósito: prioriza pasar por huecos estrechos a costa de menos margen de centrado.
- **Mantener siempre global > local**: el plan global traza rutas con más holgura (0.40) y el local hace el seguimiento fino (0.30).

### 6. Modelo del robot: footprint poligonal → `robot_radius` (circular)

Se reemplazó el footprint poligonal por un **modelo circular** (`robot_radius`) en ambos costmaps. Motivo: `base_link` está casi en la parte de atrás del robot (el polígono real iba de `x=-0.06` a `x=0.28`), así que el centro de rotación está corrido hacia atrás y al girar en el lugar el frente barre un arco grande. El polígono no representaba ese barrido; el círculo centrado en `base_link` sí cubre el peor caso de rotación.

```yaml
robot_radius: 0.33
# footprint: "[ [-0.06, -0.245], [-0.06, 0.24], [0.28, 0.24], [0.28, -0.245] ]"  # rectángulo real, descartado
```

> [!NOTE]
> El modelo circular es más conservador (infla en todas las direcciones) pero elimina los roces al girar que el footprint poligonal no anticipaba.

### 7. Velocidades y aceleraciones (DWB controller)

Además de bajar `max_vel_theta` (punto 2), se ajustaron límites de velocidad y aceleración para suavizar la dinámica:

```yaml
min_vel_x: 0.0        # forward-only en crucero: DWB no muestrea reversa. El escape de
                      # acuñamientos lo hace el recovery BackUp del BT (consciente de colisiones).
                      # No rehabilitar reversa (-0.10): DWB la elegía para tramos largos ("reversa de crucero").
max_vel_x: 0.18
max_vel_theta: 0.4    # ver punto 2
max_speed_xy: 0.18
acc_lim_x: 1.8        # tiempo a max_vel_x: 0.18/1.8 = 0.1s
acc_lim_theta: 1.5    # bajado de 2.2 junto con max_vel_theta
```

### 8. Pesos del crítico DWB

El robot seguía la ruta aunque hubiera obstáculos porque `BaseObstacle.scale` era demasiado bajo. Se subió y se redujeron los pesos de alineación/seguimiento de path para darle más peso relativo a evitar obstáculos:

```yaml
BaseObstacle.scale: 0.5   # antes: 0.2
PathAlign.scale: 24.0     # antes: 32.0
PathDist.scale: 24.0      # antes: 32.0
```

### 9. Capa de obstáculos del costmap local y retención de obstáculos delgados

El XV-11 (1°/rayo) detecta obstáculos chicos (~5 cm, p. ej. patas de silla) de forma **intermitente**. Sin retención, el raytracing los borraba entre escaneos y parpadeaban dentro/fuera del costmap. La palanca preferida en este entorno es `observation_persistence`, que mantiene una celda marcada como ocupada durante N segundos aunque el láser ya no la vea.

```yaml
observation_persistence: 6.0   # local; subido de 1.5 (global: 15.0)
raytrace_max_range: 2.1   # rayos lejanos no borran celdas; reduce falso borrado de patas de silla
raytrace_min_range: 0.17  # XV-11 no confiable bajo \~15cm; lecturas menores no borran
obstacle_max_range: 2.0   # = radio del costmap local 4×4
obstacle_min_range: 0.17  # bajo \~15cm es ruido o el propio robot
```

> [!NOTE]
> **Implicancia de diseño:** el robot asume un entorno *estático*. Si un objeto se mueve, su rastro persiste 6 s (local) / 15 s (global) como obstáculo fantasma antes de limpiarse. Es un trade-off aceptado: este robot **no está pensado para esquivar gente u objetos móviles en tiempo real**; prioriza no chocar contra obstáculos finos fijos.

### 10. Planner server y frecuencia del costmap global

```yaml
# planner_server
tolerance: 0.75   # antes: 0.5m; NavFn busca una celda libre hasta 0.75m del frontier.
                  # Necesario porque los frontiers caen dentro de la zona inflada.
# global_costmap
update_frequency: 2.0   # antes: 1.0 Hz; incorpora nuevos obstáculos más rápido al plan global
```

> [!NOTE]
> El síntoma asociado era el "No frontiers found" prematuro: la inflación bloqueaba el acceso a frontiers válidos. Subir la tolerancia del planner (junto con global inflation > local) permite que NavFn alcance frontiers cercanos a obstáculos.

---

## Resumen de cambios

| Parámetro | Antes | Después | Archivo |
| --- | --- | --- | --- |
| `transform_tolerance` | 0.3 (default) | 0.5 | nav2 (ambos costmaps) |
| `max_vel_theta` | 0.6 rad/s | 0.4 rad/s | nav2 (DWB controller) |
| `angle_variance_penalty` | 1.0 | 0.4 | slam_toolbox |
| `distance_variance_penalty` | 0.5 | 0.3 | slam_toolbox |
| `minimum_travel_heading` | 0.1 rad | 0.3 rad | slam_toolbox |
| `minimum_travel_distance` | 0.1 m | 0.15 m | slam_toolbox |
| `cost_scaling_factor` (local) | 2.5 | 1.2 | nav2 (local_costmap) |
| `cost_scaling_factor` (global) | 2.0 | 1.2 | nav2 (global_costmap) |
| modelo del robot | footprint poligonal | `robot_radius: 0.33` | nav2 (ambos costmaps) |
| `inflation_radius` (global) | 0.30 | 0.40 | nav2 (global_costmap) |
| `min_vel_x` | -0.10 | 0.0 (forward-only) | nav2 (DWB) |
| `acc_lim_theta` | 2.2 | 1.5 | nav2 (DWB) |
| `BaseObstacle.scale` | 0.2 | 0.5 | nav2 (DWB) |
| `PathAlign.scale` / `PathDist.scale` | 32.0 | 24.0 | nav2 (DWB) |
| `observation_persistence` (local) | 1.5 | 6.0 | nav2 (local obstacle_layer) |
| `tolerance` (planner) | 0.5 | 0.75 | nav2 (planner_server) |
| `update_frequency` (global) | 1.0 Hz | 2.0 Hz | nav2 (global_costmap) |

---

## Compromiso entre calidad de mapa vs. estabilidad de pose

Existe una tensión inherente en modo `mapping` que vale documentar: SLAM Toolbox necesita corregir agresivamente para mapear bien (objetivo de exploración), pero esas mismas correcciones introducen saltos en `map → odom` que degradan la navegación. Los ajustes anteriores desplazan el equilibrio hacia mayor estabilidad de pose durante la navegación, a costa de aceptar correcciones de mapa más conservadoras.

Una alternativa explorada conceptualmente pero no implementada es usar modo `localization` una vez terminado el mapa, que elimina las correcciones continuas. Sin embargo, en la práctica produjo saltos en la pose inicial y conflictos en el árbol de TF, por lo que se descartó para este proyecto.
