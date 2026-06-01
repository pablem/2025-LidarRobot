# Configuración de exploración autónoma y retorno a base

Cambios realizados respecto a la configuración original de Nav2 + explore_lite.

## Resumen de problemas resueltos

| Problema | Causa raíz | Solución |
|---|---|---|
| Robot chocaba con obstáculos | `BaseObstacle.scale` demasiado bajo; costmap global lento | Subir scale, aumentar `update_frequency` |
| Robot rozaba al girar en el lugar | Footprint poligonal no modela el barrido del frente (centro de rotación corrido) | Modelo circular `robot_radius: 0.33` |
| Obstáculos delgados (patas de silla) parpadeaban | Raytrace borraba celdas entre scans intermitentes del XV-11 | `observation_persistence` (6s local / 15s global), `raytrace_min_range: 0.17` |
| "No frontiers found" prematuro | Inflation bloqueaba acceso a frontiers | Inflation global > local pero ≥ robot_radius; `planner tolerance: 0.75` |
| Robot exploraba zonas donde no cabe | `min_frontier_size` demasiado bajo | Subir a 0.75m |
| Apuntaba a fronteras grandes lejanas cruzando zonas sin mapear | `gain_scale` dominaba sobre `potential_scale` | Volver a `potential > gain` (exploración incremental) |
| Robot se quedaba quieto al terminar exploración | Sin detección de inactividad en `return_to_base` | Suscripción a `/navigate_to_pose/_action/status` |
| Robot chocaba en esquinas al girar | Correcciones angulares bruscas de SLAM + `transform_tolerance` insuficiente con delay negativo de TF | `angle_variance_penalty` bajado, `transform_tolerance: 0.5` |

---

## `navegation2_params_waffle_mod.yaml`

### Modelo del robot: footprint → `robot_radius` (local y global costmap)

Se reemplazó el footprint poligonal por un **modelo circular** (`robot_radius`).
Motivo: `base_link` está casi en la parte de atrás del robot (el polígono real iba
de `x=-0.06` a `x=0.28`), así que el centro de rotación está corrido hacia atrás y
al girar en el lugar el frente barre un arco grande. El polígono no representaba
ese barrido; el círculo centrado en `base_link` sí cubre el peor caso de rotación.

```yaml
robot_radius: 0.33
# footprint: "[ [-0.06, -0.245], [-0.06, 0.24], [0.28, 0.24], [0.28, -0.245] ]"  # rectángulo real, descartado
```

> El modelo circular es más conservador (infla en todas las direcciones) pero
> elimina los roces al girar que el footprint poligonal no anticipaba.

### Velocidades y aceleraciones (DWB controller)

```yaml
min_vel_x: 0.0        # forward-only en crucero: DWB no muestrea reversa. El escape de
                      # acuñamientos lo hace el recovery BackUp del BT (consciente de colisiones).
                      # No rehabilitar reversa (-0.10): DWB la elegía para tramos largos ("reversa de crucero").
max_vel_x: 0.18
max_vel_theta: 0.4    # bajado de 0.6 → error de giro pasa de ~10° a ~7°
max_speed_xy: 0.18
acc_lim_x: 1.8        # tiempo a max_vel_x: 0.18/1.8 = 0.1s
acc_lim_theta: 1.5    # bajado de 2.2 junto con max_vel_theta
```

### DWB

```yaml
BaseObstacle.scale: 0.5   # antes 0.2 → demasiado bajo, seguía ruta aunque hubiera obstáculos
PathAlign.scale: 24.0     # antes 32.0; reducido para dar más peso relativo a BaseObstacle
PathDist.scale: 24.0      # ídem
```

### Costmap local — obstacle_layer

```yaml
observation_persistence: 6.0   # local; subido de 1.5. El XV-11 (1°/rayo) ve obstáculos
                               # chicos (~5cm) de forma intermitente; retenerlos evita que el
                               # raytracing los borre entre escaneos.
raytrace_max_range: 2.1   # rayos lejanos no borran celdas; reduce falso borrado de patas de silla
raytrace_min_range: 0.17  # XV-11 no confiable bajo ~15cm; lecturas menores no borran
obstacle_max_range: 2.0   # = radio del costmap local 4×4
obstacle_min_range: 0.17  # bajo ~15cm es ruido o el propio robot
```

#### Tiempo de retención de obstáculos — NO interactúa con objetos en movimiento

`observation_persistence` mantiene una celda marcada como ocupada durante N
segundos aunque el láser ya no la vea, en vez de borrarla con el siguiente raytrace.
Es la palanca preferida en este entorno porque el XV-11 detecta obstáculos delgados
de forma intermitente y, sin persistencia, parpadeaban dentro/fuera del costmap.

**Implicancia de diseño**: el robot asume un entorno *estático*. Si un objeto se
mueve, su rastro persiste 6 s (local) / 15 s (global) como obstáculo fantasma antes
de limpiarse. Es un trade-off aceptado: este robot **no está pensado para esquivar
gente u objetos móviles en tiempo real**; prioriza no chocar contra obstáculos finos
fijos (patas de silla, mesas).

### Inflation (local y global) — relación con `robot_radius`

```yaml
# local_costmap
inflation_layer:
  inflation_radius: 0.30
  cost_scaling_factor: 1.2

# global_costmap
inflation_layer:
  inflation_radius: 0.40   # > local; deja goals/frontiers menos pegados a obstáculos
  cost_scaling_factor: 1.2 # factor bajo = gradiente extendido; el planner prefiere el centro del pasillo.
                           # valores altos (>2) dan gradiente abrupto: el robot puede raspar esquinas.
```

**Relación `inflation_radius` / `robot_radius` (= 0.33 con modelo circular):**

- El gradiente de inflación útil aparece entre el radio del robot y `inflation_radius`.
  Si `inflation_radius ≤ robot_radius`, no hay zona de gradiente: las celdas pasan de
  letales a libres sin transición y `cost_scaling_factor` no tiene efecto.
- Regla práctica: **`inflation_radius` debería ser ≥ `robot_radius`** para que exista
  ese "espectro" de costes.
- El **local** está en `0.30`, ligeramente por debajo de `robot_radius` (0.33).
Reducirlo prioriza pasar por huecos
  estrechos a costa de menos margen de centrado;
- Mantener siempre **global > local**: el plan global traza rutas con más holgura y el
  local hace el seguimiento fino.

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
costmap_topic: map       # mapa crudo de slam_toolbox (no el costmap inflado de Nav2)
visualize: true          # publica fronteras en /explore/frontiers para RViz

planner_frequency: 0.10  # Hz — 10s entre evaluaciones; da tiempo a que el costmap se actualice
progress_timeout: 90.0   # s sin avance → blacklist del goal

# Relación potential_scale / gain_scale determina si prefiere fronteras cercanas o grandes:
# Con potential_scale > gain_scale → prefiere cercanas; con gain_scale > potential_scale → grandes/lejanas.
# Se volvió a potential > gain a propósito: con gain dominante el robot apuntaba a
# fronteras grandes lejanas a través de obstáculos aún no mapeados. Prefiriendo
# cercanas explora de forma incremental y mapea el camino antes de avanzar.
potential_scale: 3.5     # penaliza distancia (prefiere fronteras cercanas)
gain_scale: 2.0          # premia fronteras grandes → evita perseguir huecos diminutos
orientation_scale: 0.0   # no implementado, dejar en 0

min_frontier_size: 0.75  # filtra aperturas chicas donde el robot no cabe (r=0.33)
```
