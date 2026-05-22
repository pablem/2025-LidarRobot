# Configuración de exploración autónoma y retorno a base

Cambios realizados respecto a la configuración original de Nav2 + explore_lite.

## Resumen de problemas resueltos

| Problema | Causa raíz | Solución |
|---|---|---|
| Robot chocaba con obstáculos | `BaseObstacle.scale` demasiado bajo; costmap global lento | Subir scale, aumentar `update_frequency` |
| Robot chocaba aunque el costmap lo permitía | Footprint subestimado | +1cm por lado al polígono |
| Obstáculos delgados (patas de silla) no persistían | Raytrace borraba celdas entre scans | `raytrace_min_range: 0.15`, `raytrace_max_range: 2.0` |
| `trans_stopped_velocity` mayor que `max_vel_x` | Error de configuración | RotateToGoal siempre activo → movimiento errático |
| "No frontiers found" prematuro | Inflation bloqueaba acceso a frontiers | Inflation global < local; `planner tolerance: 0.75` |
| Robot exploraba zonas donde no cabe | `min_frontier_size` demasiado bajo | Subir a 1.0m |
| Robot prefería fronteras cercanas chicas | `potential_scale` dominaba sobre `gain_scale` | Invertir proporción |
| Robot se quedaba quieto al terminar exploración | Sin detección de inactividad en `return_to_base` | Suscripción a `/navigate_to_pose/_action/status` |
| Robot chocaba en esquinas al girar | Correcciones angulares bruscas de SLAM + `transform_tolerance` insuficiente con delay negativo de TF | `angle_variance_penalty` bajado, `transform_tolerance: 0.5` |

---

## `navegation2_params_waffle_mod.yaml`

### Footprint (local y global costmap)

```yaml
# Polígono en base_link: atrás, izquierda, derecha, adelante
# Se agregó 1cm por lado respecto a medición original porque el robot chocaba
# aun cuando el costmap indicaba que el paso era libre.
footprint: "[ [-0.06, -0.245], [-0.06, 0.24], [0.28, 0.24], [0.28, -0.245] ]"
```

### Velocidades y aceleraciones (DWB controller)

```yaml
min_vel_x: -0.05      # negativo habilita marcha atrás lenta; 20% del rango → poco muestreado
max_vel_x: 0.18
max_vel_theta: 0.6
max_speed_xy: 0.18
acc_lim_x: 1.8        # tiempo a max_vel_x: 0.18/1.8 = 0.1s
acc_lim_theta: 2.2    # tiempo a max_vel_theta: 0.6/2.2 = 0.27s
decel_lim_x: -1.8
decel_lim_theta: -2.2

# CRÍTICO: debe ser < max_vel_x. Si es mayor, RotateToGoal considera al robot
# siempre "detenido" y activa modo rotación pura en mitad de la trayectoria.
trans_stopped_velocity: 0.05
```

### Recovery server (velocidades de spin alineadas con el controlador)

```yaml
max_rotational_vel: 0.7   # igual a max_vel_theta; evita tirón brusco al entrar en recovery
min_rotational_vel: 0.3
rotational_acc_lim: 2.0   # igual a acc_lim_theta
```

### Progress checker

```yaml
required_movement_radius: 0.3   # antes 0.5m → muy exigente en espacios estrechos
movement_time_allowance: 12.0   # antes 10s
```

### Críticos DWB

```yaml
BaseObstacle.scale: 0.5   # antes 0.2 → demasiado bajo, seguía ruta aunque hubiera obstáculos
PathAlign.scale: 24.0     # antes 32.0; reducido para dar más peso relativo a BaseObstacle
PathDist.scale: 24.0      # ídem
```

### Costmap local — obstacle_layer

```yaml
raytrace_max_range: 2.0   # rayos lejanos no borran celdas; reduce falso borrado de patas de silla
raytrace_min_range: 0.15  # XV-11 no confiable bajo 15cm
obstacle_max_range: 2.0
obstacle_min_range: 0.15
```

### Inflation (local y global)

```yaml
inflation_layer:
  inflation_radius: 0.30
  cost_scaling_factor: 1.2   # factor bajo = gradiente extendido; planner prefiere el centro del pasillo
                              # valores altos (>2) dan gradiente abrupto: el robot puede raspar esquinas
```

### transform_tolerance (local y global costmap)

```yaml
transform_tolerance: 0.5   # default 0.3s era insuficiente; TF map→odom tiene delay sistemático de ~0.25s
```

### Planner server

```yaml
tolerance: 0.75   # antes 0.5m; NavFn busca una celda libre hasta 0.75m del frontier
                  # necesario porque los frontiers caen dentro de la zona inflada
```

### Global costmap — frecuencia de actualización

```yaml
update_frequency: 2.0   # antes 1.0 Hz; incorpora nuevos obstáculos más rápido al plan global
```

---

## `mapper_params_online_async.yaml`

Parámetros ajustados para reducir correcciones bruscas de `map→odom` durante rotaciones. La odometría filtrada por EKF es más confiable que el scan matcher en giros.

```yaml
minimum_travel_distance: 0.15   # mínima traslación para procesar un scan nuevo
minimum_travel_heading: 0.3     # mínima rotación (~17°) para procesar un scan; reduce a ~5 correcciones por giro de 90°

# Scan Matcher — valores bajos = el optimizador penaliza más desviarse del odom
distance_variance_penalty: 0.3  # antes 0.5
angle_variance_penalty: 0.4     # antes 1.0; el más importante para evitar saltos en rotación
```

---

## `m-explore-ros2/explore/config/params.yaml`

```yaml
return_to_init: false    # explore_lite usa frame map; return_to_base.py maneja el retorno a odom(0,0,0)

planner_frequency: 0.33  # antes 0.1 Hz → muy lento para re-evaluar tras blacklistear un frontier

# Relación potential_scale / gain_scale determina si prefiere fronteras cercanas o grandes:
# costo = potential_scale × distancia − gain_scale × tamaño
# Con potential_scale > gain_scale → prefiere cercanas (comportamiento original, problemático)
# Con gain_scale > potential_scale → prefiere grandes y lejanas
potential_scale: 1.0     # antes 3.0
gain_scale: 3.0          # antes 1.0

min_frontier_size: 1.0   # antes 0.5m; con robot de 0.49m de ancho, frontiers <1m llevan a zonas inaccesibles
```

---

## `return_to_base.py` — detección de inactividad

El script original solo tenía un timer fijo. Problema: el robot terminaba de explorar y se quedaba quieto hasta que el timer disparaba.

Se agregó detección de fin de exploración por inactividad de Nav2:

```python
# Suscripción al topic de estado de la action server de Nav2
'/navigate_to_pose/_action/status'  →  GoalStatusArray

# Lógica:
# - Si hay goals STATUS_ACCEPTED o STATUS_EXECUTING → actualizar _last_active_time
# - Cada 2s: si han pasado min_exploration_time Y Nav2 lleva idle_timeout sin goals activos
#   → disparar retorno (misma lógica que el timer)
```

Parámetros nuevos expuestos en el launch file:

```python
'min_exploration_time': 60.0,   # no activar idle antes de este tiempo (evita falso positivo al arranque)
'idle_timeout': 8.0,            # segundos sin goals Nav2 → exploración terminada
```

Los tres triggers (timer, idle, batería) compiten; el primero en disparar setea `_returning = True` y los demás no hacen nada.
