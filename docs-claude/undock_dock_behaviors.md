## Por qué Nav2 no resuelve la salida/entrada al dock

El robot vive sobre una **base de carga adosada a una pared**. La primera y última
pose del ciclo son ciegas para Nav2:

- **Undock**: el robot arranca pegado a la pared, con el costmap inflado encima suyo.
  Cualquier `NavigateToPose` rechaza el plan (ya está "dentro" de un obstáculo).
- **Dock final**: la pose exacta del dock cae sobre la pared inflada. El goal Nav2
  se manda a una pose **delante** del dock (`dock_x_offset`) y el último tramo
  —retroceder hasta tocar— queda fuera del planner.

Ambos tramos necesitan una maniobra **recta, open-loop, sin chequeo de costmap**.

El lazo abierto publica `Twist` crudo en `cmd_vel` (vía `twist_mux`, prioridad
navigation) durante un tiempo calculado (`dist/speed`), **ignorando el costmap**, así
que siempre sale/entra del dock.`twist_mux` se republica a `publish_rate = 20 Hz` por un tema de timeout que impone el control manager.

> Observación: la distancia real depende en parte del nivel de batería, pero
> undock y dock son simétricos (ambos se ejecutan con una carga similar de batería), así que
> sus desvíos se mueven parejos y en gran medida se cancelan.


| Parámetro | Default | Descripción |
|---|---|---|
| `undock_dist` | 0.40 m | Distancia a avanzar para salir del dock |
| `undock_speed` | 0.10 m/s | Velocidad de la maniobra |
| `startup_delay` | 1.0 s | Espera a que `twist_mux`/controller estén listos |
| `publish_rate` | 20 Hz | Republicación del Twist (timeout twist_mux 0.5 s) |

## Calibración del `dock_yaw_offset`

Tras un ciclo completo, medir el yaw final y compensar con el signo opuesto:

```bash
ros2 run tf2_ros tf2_echo map base_link
```

Si el yaw medido es `-0.10 rad`, setear `dock_yaw_offset: 0.10` en `explore.launch.py`.
Promediar 2-3 corridas; si el desvío no es repetible, no compensa calibrarlo.
