# Costmap2DClient — adaptador de mapa para explore_lite

**Archivo:** [m-explore-ros2/explore/src/costmap_client.cpp](../m-explore-ros2/explore/src/costmap_client.cpp)  
**Header:** [m-explore-ros2/explore/include/explore/costmap_client.h](../m-explore-ros2/explore/include/explore/costmap_client.h)  
**Namespace:** `explore`  
**Clase:** `Costmap2DClient`

---

## Responsabilidad

Suscribe al mapa de Nav2 (publicado como `nav_msgs/OccupancyGrid` + actualizaciones parciales `OccupancyGridUpdate`) y lo mantiene en un objeto `nav2_costmap_2d::Costmap2D` local que `FrontierSearch` puede leer de forma thread-safe. También provee la pose del robot en el frame global via TF.

---

## Constructor (bloqueante)

El constructor **bloquea** hasta que:
1. Se recibe al menos un mensaje en `costmap_topic` → costmap inicializado.
2. El transform `global_frame → robot_base_frame` está disponible en TF.

Esto es equivalente al `waitForMessage` de ROS 1. Es el motivo del log:

```
[explore-1] [INFO] Waiting for costmap to become available, topic: map
```

Los ~6 segundos de espera observados en los logs corresponden al tiempo que tardó Nav2 en publicar el primer costmap (Nav2 arranca con `TimerAction(5s)` en `slam_nav.launch.py`).

---

## Suscripciones

| Topic | Tipo | Callback |
|-------|------|---------|
| `costmap_topic` (default: `map`) | `nav_msgs/OccupancyGrid` | `updateFullMap()` |
| `costmap_updates_topic` (default: `map_updates`) | `map_msgs/OccupancyGridUpdate` | `updatePartialMap()` |

El frame global (`global_frame_`) se extrae del header del primer mensaje del costmap.

---

## `updateFullMap(msg)`

Redimensiona el costmap interno con `resizeMap()` y copia todos los valores pasando por la tabla de traducción:

```
OccupancyGrid valor [-1..100] → cost_translation_table → valor interno [0..255]
```

Tabla de traducción especial:
- `0` → `0` (FREE_SPACE)
- `99` → `253` (INSCRIBED)  
- `100` → `254` (LETHAL)
- `-1` (uint8=255) → `255` (NO_INFORMATION)

Cada resize del costmap aparece como `StaticLayer: Resizing costmap to X x Y` en los logs de Nav2 — son las actualizaciones que `updateFullMap` procesa cuando el SLAM extiende el mapa.

---

## `updatePartialMap(msg)`

Aplica actualizaciones incrementales a una región del costmap sin redimensionar. Si la actualización excede los límites del costmap actual, copia solo la parte que cabe y emite un WARN.

---

## `getRobotPose()`

Transforma la pose del robot (`robot_base_frame`) al frame global (`global_frame_`) via `tf_->transform()`. Tiene throttled error logging (1s) para los casos típicos de TF: Lookup, Connectivity, Extrapolation. Retorna `Pose()` vacía en caso de error (lo cual fuerza a `FrontierSearch::searchFrom` a fallar con "Robot out of costmap bounds").

---

## Parámetros

| Parámetro | Default | Descripción |
|-----------|---------|-------------|
| `costmap_topic` | `costmap` | Topic del mapa (en este robot: `map`) |
| `costmap_updates_topic` | `costmap_updates` | Topic de actualizaciones parciales (en este robot: `map_updates`) |
| `robot_base_frame` | `base_link` | Frame del robot |
| `transform_tolerance` | 0.3 s | Timeout para lookups TF |

**Config del robot** (`params.yaml`): `costmap_topic: map`, `costmap_updates_topic: map_updates`, `transform_tolerance: 0.3`.
