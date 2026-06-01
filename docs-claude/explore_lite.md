## Idea general

explore_lite hace **exploración basada en fronteras**: una *frontera* es el límite
entre espacio libre y espacio desconocido del mapa. El nodo busca todas las
fronteras, elige la "mejor" por una función de costo y le manda al robot un goal
a esa frontera vía Nav2. Al llegar (o al crecer el mapa) re-evalúa y repite, hasta
que no quedan fronteras → exploración terminada.

No mantiene mapa propio: consume el `OccupancyGrid` de SLAM/Nav2 y delega toda la
navegación y evitación de obstáculos a Nav2.

```
SLAM/Nav2 (/map) ──► Costmap2DClient ──► FrontierSearch ──► Explore.makePlan()
                                                                  │
                                                                  ▼
                                                    NavigateToPose (action) ──► Nav2
```

---

## Tres componentes

### 1. `Costmap2DClient` — adaptador de mapa (`namespace explore`)

Suscribe al mapa y lo mantiene en un `nav2_costmap_2d::Costmap2D` local, thread-safe,
que `FrontierSearch` puede leer mientras se actualiza en paralelo.

- **Constructor bloqueante**: espera (a) el primer mensaje del costmap y (b) el
  transform `global_frame → robot_base_frame` en TF. De ahí el log
  `Waiting for costmap to become available, topic: map` (~6 s, lo que tarda Nav2
  en publicar el primer costmap; arranca con `TimerAction(5s)` en `slam_nav.launch.py`).
- **Suscripciones**: `costmap_topic` (`map`, `OccupancyGrid`, callback `updateFullMap`)
  y `costmap_updates_topic` (`map_updates`, `OccupancyGridUpdate`, callback
  `updatePartialMap`). El frame global se toma del header del primer mensaje.
- `updateFullMap` redimensiona y copia todo pasando por la tabla de traducción
  (ver abajo). Cada resize aparece como `StaticLayer: Resizing costmap to X x Y`
  en los logs de Nav2.
- `updatePartialMap` aplica updates incrementales sin redimensionar.
- `getRobotPose()` transforma `base_link → global_frame` vía TF. Si falla
  (Lookup/Connectivity/Extrapolation) retorna pose vacía → `searchFrom` falla con
  "Robot out of costmap bounds".

### 2. `FrontierSearch` — búsqueda y ranking (`namespace frontier_exploration`)

`searchFrom(position)`:
```
1. Verifica que el robot esté dentro del costmap.
2. Lock del mutex del costmap (thread-safe).
3. BFS 4-conectado desde la celda libre más cercana al robot, visitando FREE_SPACE.
   Si una vecina es NO_INFORMATION con ≥1 vecino FREE_SPACE → celda fronteriza
   → buildNewFrontier() (BFS 8-conectado que expande la frontera completa).
4. Descarta fronteras con  size × resolution < min_frontier_size.
5. Calcula costo de cada frontera y ordena ascendente (menor costo = mejor).
```

Estructura `Frontier`: `size`, `min_distance`, `cost`, `initial`, `centroid`,
`middle` (punto más cercano al robot), `points`. **El goal enviado a Nav2 es
`frontier.centroid`.**

**Función de costo** (clave para el comportamiento):
```
cost = potential_scale × min_distance × resolution    (penaliza fronteras lejanas)
     − gain_scale      × size         × resolution    (premia fronteras grandes)
```
La relación `potential_scale` vs `gain_scale` decide si prefiere fronteras
**cercanas** (potential > gain) o **grandes/lejanas** (gain > potential). Ver la
config actual y su justificación en [explore_settings_and_script.md](explore_settings_and_script.md).

### 3. `Explore` — nodo orquestador (`namespace explore`, `rclcpp::Node`)

Ciclo `makePlan()` (cada `1/planner_frequency` s):
```
makePlan()
  ├─ getRobotPose()                         pose actual
  ├─ searchFrom(pose)                       fronteras ordenadas por costo
  ├─ sin fronteras → stop(finished) → returnToInitialPose() si return_to_init
  ├─ filtra fronteras en blacklist
  ├─ mismo goal sin progreso > progress_timeout → blacklist → re-makePlan()
  ├─ mismo goal con progreso → espera
  └─ goal nuevo → async_send_goal(navigate_to_pose) → reachedGoal()
```

`reachedGoal`: `SUCCEEDED` → `makePlan()` inmediato; `ABORTED` → blacklistea el goal;
`CANCELED` → no re-planifica (pausado externamente).

**Blacklist**: se agrega una frontera si Nav2 aborta o si vence `progress_timeout`
sin avance. `goalOnBlacklist()` compara con tolerancia de 5 celdas × resolución.
Vive en memoria, no se persiste entre sesiones.

**Control externo** (topic `explore/resume`, `std_msgs/Bool`):
```bash
ros2 topic pub /explore/resume std_msgs/msg/Bool "data: false" --once   # pausar
ros2 topic pub /explore/resume std_msgs/msg/Bool "data: true"  --once   # reanudar
```
Al reanudar, `resuming_` evita un blacklist inmediato por `progress_timeout`.

**Retorno al origen**: si `return_to_init: true`, al terminar manda goal a la pose
inicial grabada en el constructor (`ExploreStatus::RETURNING_TO_ORIGIN` →
`RETURNED_TO_ORIGIN`). En este robot está en `false`: el retorno a `odom(0,0,0)` lo
maneja `return_to_base.py` (ver [explore_settings_and_script.md](explore_settings_and_script.md)).

---

## Topics y actions

| Nombre | Tipo | Dir | Descripción |
|---|---|---|---|
| `map` (`costmap_topic`) | `OccupancyGrid` | sub | Costmap que consume `Costmap2DClient` |
| `map_updates` | `OccupancyGridUpdate` | sub | Actualizaciones parciales |
| `explore/resume` | `std_msgs/Bool` | sub | `true` reanuda, `false` pausa |
| `explore/status` | `explore_lite_msgs/ExploreStatus` | pub | Estado (QoS transient_local) |
| `explore/frontiers` | `visualization_msgs/MarkerArray` | pub | Markers RViz (si `visualize: true`) |
| `navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | action client | Goals a Nav2 |

---

## Tabla de traducción de valores del costmap

`Costmap2DClient` mapea `OccupancyGrid` (-1..100) → valores internos de
`nav2_costmap_2d` (0..255). `FrontierSearch` opera sobre los valores internos:

| OccupancyGrid | Interno | Significado |
|---|---|---|
| 0 | 0 (`FREE_SPACE`) | Libre |
| 1–98 | 2–252 | Ocupado parcial |
| 99 | 253 (`INSCRIBED`) | Inscribed obstacle |
| 100 | 254 (`LETHAL`) | Obstáculo letal |
| -1 (255) | 255 (`NO_INFORMATION`) | Desconocido |

---

## Notas de operación

- Las paradas frecuentes (~3 s) son normales: explore_lite actualiza su goal
  cuando el mapa crece y aparece una frontera mejor.
- `BehaviorTree tick rate exceeded` en Raspberry Pi es esperado bajo carga; no
  afecta la navegación.
- Ctrl+C cancela el goal en curso limpiamente.
- En una sesión corta el costmap creció de 64×108 a 113×127 celdas (0.05 m/px),
  mapeando ~5.6×6.4 m.
