# Explore — nodo principal de exploración autónoma

**Archivo:** [m-explore-ros2/explore/src/explore.cpp](../m-explore-ros2/explore/src/explore.cpp)  
**Header:** [m-explore-ros2/explore/include/explore/explore.h](../m-explore-ros2/explore/include/explore/explore.h)  
**Namespace:** `explore`  
**Clase:** `Explore : public rclcpp::Node`

---

## Responsabilidad

Orquesta la exploración autónoma: llama a `FrontierSearch` para obtener fronteras, elige la de menor costo, y envía goals a Nav2 via la action `navigate_to_pose`. Se re-evalúa a la frecuencia `planner_frequency`.

---

## Topics y actions

| Nombre | Tipo | Dirección | Descripción |
|--------|------|-----------|-------------|
| `map` (o el de `costmap_topic`) | `nav_msgs/OccupancyGrid` | sub | Costmap que consume `Costmap2DClient` |
| `map_updates` | `map_msgs/OccupancyGridUpdate` | sub | Actualizaciones parciales del costmap |
| `explore/resume` | `std_msgs/Bool` | sub | `true` reanuda, `false` pausa la exploración |
| `explore/status` | `explore_lite_msgs/ExploreStatus` | pub | Estado actual (QoS transient_local) |
| `explore/frontiers` | `visualization_msgs/MarkerArray` | pub | Fronteras visualizadas en RViz (solo si `visualize: true`) |
| `navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | action client | Envía goals a Nav2 |

---

## Ciclo principal: `makePlan()`

```
makePlan() [llamado cada 1/planner_frequency segundos]
  ├── costmap_client_.getRobotPose()        ← pose actual del robot
  ├── search_.searchFrom(pose.position)     ← lista de fronteras ordenadas por costo
  ├── Si no hay fronteras → stop(finished=true) → returnToInitialPose() si return_to_init
  ├── Filtra fronteras en blacklist
  ├── Si mismo goal que antes y sin progreso > progress_timeout → blacklist → makePlan()
  ├── Si mismo goal con progreso → no hace nada (espera)
  └── Si goal nuevo → async_send_goal(navigate_to_pose)
        └── result_callback → reachedGoal()
```

### `reachedGoal(result, frontier_goal)`

| Resultado Nav2 | Acción |
|----------------|--------|
| `SUCCEEDED` | Llama `makePlan()` inmediatamente para buscar siguiente frontera |
| `ABORTED` | Agrega `frontier_goal` a `frontier_blacklist_`, retorna sin re-plan (ya hay otro goal en camino) |
| `CANCELED` | Retorna sin re-plan (la exploración fue pausada externamente) |

---

## Blacklist de fronteras

- Se agrega una frontera si: Nav2 aborta el goal, o si `progress_timeout` se cumple sin avance.
- `goalOnBlacklist()` compara con tolerancia de 5 celdas × resolución del costmap.
- La blacklist no se persiste entre sesiones (vive en `frontier_blacklist_` en memoria).

---

## Control externo via topic

```bash
# Pausar exploración
ros2 topic pub /explore/resume std_msgs/msg/Bool "data: false" --once

# Reanudar
ros2 topic pub /explore/resume std_msgs/msg/Bool "data: true" --once
```

Al reanudar, `resuming_` se pone en `true` por un ciclo para evitar que `progress_timeout` blacklistee el goal inmediatamente.

---

## Retorno al origen

Si `return_to_init: true` (valor por defecto en este robot), al finalizar la exploración el nodo envía un goal con la pose inicial grabada en el constructor. Se publica `ExploreStatus::RETURNING_TO_ORIGIN` mientras navega y `RETURNED_TO_ORIGIN` al llegar.

---

## Parámetros

| Parámetro | Default | Descripción |
|-----------|---------|-------------|
| `planner_frequency` | 1.0 Hz | Frecuencia del timer de `makePlan` |
| `progress_timeout` | 30.0 s | Segundos sin avance antes de blacklistear |
| `potential_scale` | 1e-3 | Peso de distancia en la función de costo |
| `gain_scale` | 1.0 | Peso del tamaño de frontera |
| `min_frontier_size` | 0.5 m | Umbral mínimo para considerar una frontera |
| `visualize` | false | Publica markers en `/explore/frontiers` |
| `return_to_init` | false | Vuelve a la pose inicial al terminar |
| `robot_base_frame` | `base_link` | Frame del robot |

**Config del robot** (`params.yaml`): `planner_frequency: 0.33`, `min_frontier_size: 0.75`, `return_to_init: true`, `potential_scale: 3.0`.

---

## Notas de operación (probado 2026-05-19)

- Las preempciones frecuentes (~3s) son normales: explore_lite actualiza su goal cuando el mapa crece y aparece una frontera mejor.
- `BehaviorTree tick rate exceeded` en Raspberry Pi es esperado bajo carga; no afecta la navegación.
- Al hacer Ctrl+C, Nav2 cancela el goal en curso limpiamente.
- El costmap creció de 64×108 a 113×127 celdas (0.05 m/px) mapeando ~5.6×6.4 m en una sesión corta.
