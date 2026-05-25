# Undock y dock — maniobras fuera del alcance de Nav2

## Por qué Nav2 no resuelve la salida/entrada al dock

El robot vive sobre una **base de carga adosada a una pared**. Eso introduce dos restricciones que Nav2 no puede satisfacer con su flujo normal:

- En arranque (undock), el robot está pegado a la pared con el costmap inflado encima suyo. Cualquier intento de `NavigateToPose` rechaza el plan: el robot ya está "dentro" de un obstáculo desde la perspectiva del planner.
- En cierre (dock final), la pose exacta del dock cae sobre la pared inflada. Nav2 no acepta un goal cuya celda destino está en la zona de inflation, por lo que el goal real (`navigate_to_pose`) se envía a una pose **delante del dock** (`dock_x_offset`), y el último tramo —retroceder hasta tocar el dock— queda por fuera del planner.

Conclusión: la primera y última pose del ciclo son ciegas para Nav2 y necesitan una maniobra "open loop" en línea recta, sin chequeo de costmap.

## Por qué DriveOnHeading y BackUp (y no `cmd_vel` directo)

`DriveOnHeading` y `BackUp` son dos behaviors del `behavior_server` de Nav2 (paquete `nav2_behaviors` en Humble). Son nodos action-server que ejecutan una maniobra **abierta** (sin planner, sin costmap):

| Behavior | Action | Goal | Uso |
|---|---|---|---|
| `DriveOnHeading` | `/drive_on_heading` | `target.x` (m), `speed` (m/s), `time_allowance` | Avanza recto en el heading actual |
| `BackUp` | `/backup` | `target.x` (m, positivo), `speed` (m/s, positivo) | Retrocede recto; el behavior invierte el signo internamente |

Internamente ambos publican `cmd_vel` a la frecuencia de `cycle_frequency` del `behavior_server` (10 Hz), monitorean `simulate_ahead_time` segundos hacia adelante en el costmap para detectar colisión inminente, y cierran el control con el odom. **Eso es justamente lo que nuestra primera versión basada en `cmd_vel` no hacía bien**: en una Raspberry Pi bajo carga, los timers de rclpy se retrasaban y `twist_mux` zerificaba el comando entre publicaciones, produciendo "tirones" cortos en lugar de una maniobra continua. El `behavior_server` corre en C++, no compite por el GIL y mantiene el comando estable.

Bonus: usar los behaviors mantiene la pila ROS-nativa (action interfaces, status reporting) sin un thread casero en Python.

## Flujo de ejecución

El stack se lanza siempre en este orden:

```
launch_robot.launch.py    # motores, lidar, ros2_control, twist_mux
slam_nav.launch.py        # slam_toolbox + Nav2 (incluye behavior_server)
explore.launch.py         # undock → explore_lite → return_to_base
```

`slam_nav` levanta el `behavior_server` con `drive_on_heading` y `backup` ya cargados — `explore.launch.py` asume que están disponibles cuando arranca undock.

## undock (`robot/scripts/undock.py`)

Nodo de vida corta que se lanza al inicio de `explore.launch.py`:

```python
self._client = ActionClient(self, DriveOnHeading, 'drive_on_heading')

# Espera startup_delay antes de mandar el goal (deja que Nav2 termine de subir)
self._start_timer = self.create_timer(self._delay, self._on_start_timer)

# En el callback:
goal.target.x = undock_dist          # ej. 0.40 m
goal.speed = undock_speed            # ej. 0.10 m/s
goal.time_allowance = Duration(sec=int(dist/speed * 3))
```

Parámetros expuestos en el launch:

| Parámetro | Default | Descripción |
|---|---|---|
| `undock_dist` | 0.40 m | Distancia a avanzar al salir del dock |
| `undock_speed` | 0.10 m/s | Velocidad de la maniobra |
| `startup_delay` | 1.0 s | Espera antes de mandar el goal — margen para que `behavior_server` esté listo |

Al recibir el resultado, el nodo llama `rclpy.shutdown()` y se apaga. `explore_lite` arranca 5 s después vía `TimerAction` en el launch, dándole tiempo de sobra al undock.

## dock (refactor in-place en `return_to_base.py`)

No es un nodo separado: se decidió mantener la orquestación (timing, retorno a base, guardado de mapa) en `return_to_base.py` y solo reemplazar el thread de `cmd_vel` por una llamada a la action `BackUp`. El flujo completo de retorno es:

```
trigger (timer / idle / batería)
  → pausar explore_lite
  → NavigateToPose a (dock_x_offset, 0, yaw=dock_yaw_offset)   ── Nav2
  → goal SUCCEEDED
  → esperar dock_startup_delay s
  → BackUp(target.x=dock_reverse_dist, speed=dock_speed)        ── behavior_server
  → guardar mapa con /slam_toolbox/serialize_map
  → shutdown
```

Parámetros relevantes:

| Parámetro | Default | Descripción |
|---|---|---|
| `dock_x_offset` | 0.40 m | Goal Nav2 delante del dock (no la pose del dock en sí) |
| `dock_yaw_offset` | 0.0 rad | Corrección de yaw del goal final — compensa desvío sistemático medido con `tf2_echo` |
| `dock_reverse_dist` | 0.45 m | Distancia que retrocede `BackUp` para pegarse al dock |
| `dock_speed` | 0.10 m/s | Velocidad del BackUp |
| `dock_startup_delay` | 1.0 s | Espera entre el `succeeded` de Nav2 y el envío del BackUp |

Fallback: si el action server `backup` no responde o rechaza el goal, se loggea un `error` y se procede directo a guardar el mapa — la persistencia del mapa pesa más que el alineamiento perfecto con el dock.

## Calibración del `dock_yaw_offset`

Después de que el robot completa el ciclo, medir el ángulo final:

```bash
ros2 run tf2_ros tf2_echo map base_link
```

Si el yaw medido es, por ejemplo, `-0.10 rad`, setear `dock_yaw_offset: 0.10` en `explore.launch.py` (el signo opuesto al desvío observado). Conviene promediar 2-3 corridas — si el desvío no es repetible, no compensa calibrarlo.
