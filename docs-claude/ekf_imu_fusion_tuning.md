
## Fuentes de `vyaw` y sus covarianzas

El EKF fusiona por **covarianza inversa**: una fuente con covarianza más chica pesa más.

| Fuente | Topic | Covarianza `vyaw` | Dónde se define | Peso ∝ 1/cov |
|---|---|---|---|---|
| Gyro Z (IMU) | `/imu/data` | `0.002` | [mpu9250driver.cpp](../mpu9250driver/src/mpu9250driver.cpp) `angular_velocity_covariance[8]` | 500 |
| Encoder (diff drive) | `/diff_cont/odom` | `0.004` | [my_controllers.yaml](../robot/config/my_controllers.yaml) `twist_covariance_diagonal[5]` | 250 |

Resultado: **IMU domina ~2:1 sobre el encoder**. En marcha normal ambos promedian (baja el ruido);
en giros rápidos —donde la rueda patina y el yaw-rate del encoder miente— la IMU manda, que es lo
deseado.

## Cambios aplicados

| Archivo | Parámetro | Antes | Ahora | Motivo |
|---|---|---|---|---|
| [my_controllers.yaml](../robot/config/my_controllers.yaml) | `twist_covariance_diagonal[5]` (vyaw) | `0.01` | `0.004` | Que el encoder pese de verdad (2:1 vs IMU) |
| [ekf.yaml](../robot/config/ekf.yaml) | `odom0_config` vyaw | `false` | `true` | Activar la fusión del yaw-rate del encoder |
| [ekf.yaml](../robot/config/ekf.yaml) | `imu0_twist_rejection_threshold` | `3.0` | **quitado** | El gate rechazaba arranques de giro rápido (ver abajo) |


## La distancia de Mahalanobis y los rejection thresholds

Cada `*_rejection_threshold` es un **portón (gate)**: descarta una medición si su
**distancia de Mahalanobis** supera el umbral. Mahalanobis = cuántas "sigmas" de distancia
hay entre lo medido y lo que el filtro esperaba (la *innovación*), normalizado por la
incertidumbre combinada. Es adimensional:

```
d = |z - ẑ| / sqrt(R + P)
```

- `z`  = medición del sensor (yaw-rate del gyro)
- `ẑ`  = predicción del filtro para esa variable
- `R`  = covarianza de la medición (`0.002` para el gyro)
- `P`  = covarianza del estado en esa variable (~`0.02`)

El gate sirve para sensores que escupen valores espurios (GPS saltando, pose de landmark
mal asociada). Con **un solo gyro y sin magnetómetro, no hay outliers reales que filtrar**.

### Por qué un gate fijo siempre queda corto en giros rápidos

La innovación crece con la velocidad de giro. En un giro de `0.40 rad/s` cuando el filtro
venía prediciendo `0`:

```
d = |0.40 - 0| / sqrt(0.002 + 0.02) = 0.40 / 0.148 ≈ 2.7
```

Despejando el yaw-rate máximo que admite cada gate en el arranque del giro (`umbral × sqrt(R+P)`):

| Gate | yaw-rate máx. admitido | Efecto |
|---|---|---|
| `0.8` | ~0.12 rad/s | Rechaza casi todo giro → **el mapa "acompañaba" el giro del robot** |
| `3.0` | ~0.44 rad/s | OK en giros lentos, **rechaza giros rápidos** |
| sin gate | ∞ | Acepta todo |

El robot puede girar a `3.4 rad/s` (límite de aceleración angular del EKF). En el arranque de un
giro brusco la innovación supera de largo los `0.44 rad/s` del gate `3.0`, rechazando justo la
medición necesaria para empezar a seguir el giro.

### Historial del gate

1. **`0.8`** (original): demasiado estricto. Síntoma: en cada giro el yaw no se actualizaba y, en
   el frame fijo, el mapa entero parecía rotar con el robot (la medición daba Mahalanobis ~2.7 y
   se rechazaba en cada ciclo).
2. **`3.0`**: corrigió los giros normales, pero seguía rechazando los arranques de giro rápido.
3. **Quitado**: decisión final. Sin outliers reales que filtrar, el gate solo estorba. Coherente
   con la fusión del encoder recién activada: en giros rápidos no queremos depender del encoder
   (patina), queremos que la IMU pase.

> **Si reaparece un glitch** (p. ej. el I2C corrompe una lectura y se ve un salto de yaw): restaurar
> un gate **generoso** (`10`–`15`) que admite todo giro real pero corta valores absurdos. No volver
> a valores chicos.

### Thresholds inertes

`imu0_pose_rejection_threshold` y `imu0_linear_acceleration_rejection_threshold` quedaron
**comentados**: solo aplican a variables que se fusionan, y de la IMU se fusiona únicamente `vyaw`
(twist). No filtraban nada. Lo mismo aplica a `odom0_pose_rejection_threshold` (de odom0 solo se
fusiona `vx`).
