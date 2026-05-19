# FrontierSearch — búsqueda y ranking de fronteras

**Archivo:** [m-explore-ros2/explore/src/frontier_search.cpp](../m-explore-ros2/explore/src/frontier_search.cpp)  
**Header:** [m-explore-ros2/explore/include/explore/frontier_search.h](../m-explore-ros2/explore/include/explore/frontier_search.h)  
**Namespace:** `frontier_exploration`

---

## Responsabilidad

Dado un costmap (`nav2_costmap_2d::Costmap2D`) y la posición actual del robot, encuentra todas las **fronteras** (límites entre espacio libre y espacio desconocido), las agrupa en estructuras `Frontier`, y las ordena por costo para que `Explore::makePlan()` elija a cuál ir.

---

## Estructura `Frontier`

```cpp
struct Frontier {
  uint32_t size;                         // número de celdas en la frontera
  double min_distance;                   // distancia mínima desde el robot
  double cost;                           // costo calculado (menor = mejor)
  geometry_msgs::msg::Point initial;     // primer punto de contacto encontrado
  geometry_msgs::msg::Point centroid;    // centro de masa de la frontera
  geometry_msgs::msg::Point middle;      // punto más cercano al robot
  std::vector<Point> points;             // todas las celdas
};
```

El goal que se envía a Nav2 es `frontier.centroid`.

---

## Algoritmo: `searchFrom(position)`

```
1. Verificar que el robot esté dentro del costmap.
2. Lock del mutex del costmap (thread-safe).
3. BFS desde la celda libre más cercana al robot:
   - Visita celdas FREE_SPACE en vecindad 4-conectada.
   - Si una celda vecina es NO_INFORMATION con al menos un vecino FREE_SPACE
     → es celda fronteriza → buildNewFrontier().
4. Descartar fronteras con size * resolution < min_frontier_size.
5. Calcular costo de cada frontera con frontierCost().
6. Ordenar por costo ascendente (menor costo = mejor opción).
7. Retornar lista ordenada.
```

### Función de costo

```
cost = potential_scale * min_distance * resolution
     - gain_scale      * size         * resolution
```

- **`potential_scale`** penaliza fronteras lejanas.
- **`gain_scale`** premia fronteras grandes (más información a ganar).
- Con los parámetros del robot (`potential_scale=3.0`, `gain_scale=1.0`), la distancia pesa 3× más que el tamaño → tiende a explorar lo más cercano primero.

---

## `buildNewFrontier(initial_cell, reference, frontier_flag)`

BFS 8-conectado desde `initial_cell` que expande la frontera mientras encuentre celdas `NO_INFORMATION` adyacentes a `FREE_SPACE`. Calcula el centroide como promedio de coordenadas y mantiene el punto más cercano al robot (`middle`).

---

## `isNewFrontierCell(idx, frontier_flag)`

Una celda es frontera si:
1. Su valor en el costmap es `NO_INFORMATION`.
2. No está ya marcada como frontera.
3. Al menos un vecino 4-conectado es `FREE_SPACE`.

---

## Traducción de valores del costmap

`Costmap2DClient` usa una tabla de traducción de 256 entradas que mapea los valores de `OccupancyGrid` (-1..100) a los valores internos de `nav2_costmap_2d`:

| OccupancyGrid | Costmap interno | Significado |
|---|---|---|
| 0 | 0 (`FREE_SPACE`) | Libre |
| 1–98 | 2–252 | Ocupado parcialmente |
| 99 | 253 (`INSCRIBED`) | Inscribed obstacle |
| 100 | 254 (`LETHAL`) | Obstáculo letal |
| -1 (255) | 255 (`NO_INFORMATION`) | Desconocido |

`FrontierSearch` opera directamente sobre los valores internos del costmap, no sobre el OccupancyGrid.

---

## Thread safety

El constructor toma `nav2_costmap_2d::Costmap2D*` y `searchFrom()` adquiere `costmap_->getMutex()` durante toda la búsqueda. Es seguro llamarlo desde el timer de `Explore` mientras `Costmap2DClient` actualiza el mapa en paralelo.
