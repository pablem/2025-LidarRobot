---
title: "Sensor de batería"
sidebar:
  order: 2
---

> [!NOTE]
> Se necesita conocer el estado de carga de batería para evitar descargas que dañen las celdas (por debajo de 3,0 V) y para informar el tiempo restante de uso.

## Rango de tensiones

Durante el desarrollo se usan paquetes [Turnigy LiPo 3S](./baterias.md) (11,1 V nominales): una de 1300 mAh y otra de 2200 mAh, ambas de 30C de descarga. Al ser una configuración 3S, el pack tiene tres celdas en serie, cada una con un rango operativo de 3,6 V (alarma de carga) a 4,2 V (carga plena). El pack completo varía entonces entre 10,8 V (descargado) y 12,6 V (cargado).

Lo relevante para el sensado es que estas baterías cuentan con un conector de balance JST-XH de 4 pines, dónde se puede medir tensión de cada acumulada por cada celda con respecto a GND. 

Al poder medir la tensión de celda permite detectar un pack desbalanceado. Sin embargo, sólo implementaremos la medición del total.

| Pin | Señal | Tensión aprox. (pack cargado) |
| --- | --- | --- |
| 1 | GND | 0 V |
| 2 | Celda 1 | ~4,2 V |
| 3 | Celda 1 + 2 | ~8,4 V |
| 4 | Celda 1 + 2 + 3 | ~12,6 V |

## Plan de trabajo

La estrategia elegida es aprovechar el ADC del ESP32-S3 que ya forma parte del proyecto, en lugar de comprar un módulo dedicado (INA226, ADS1115). 

```
Conector de balance (10,8–12,6 V)
        │
        ▼
Divisores resistivos  (escalan el valor a ≤ 3,3 V)
        │
        ▼
ADC1, ESP32-S3  (12 bits)
        │  
        ▼
Serial (hardware interface ros2_control)
        │
        ▼
ROS 2  → topic /battery_state (sensor_msgs/BatteryState)
```

El ESP32 ya es el dueño del puerto serie y atiende los comandos de motores y encoders; agregar la lectura de batería es solo un comando más en el mismo protocolo, sin hardware adicional.

## Elección de las resistencias

- Hay que elegir las relaciones para que quede por debajo del límite de 3,3 V del ADC.
- El ADC del ESP32 es más fiable lejos de los extremos (rango lineal), estrictamente hablando: 150 mV ~ 2450 mV para el modo `ADC_11db`
- El consumo debe ser poco ≈ 12,6 V / 57.000 Ω ≈ 0,22 mA.
- La impedancia de Thévenin vista por el ADC ($R_{th} = \frac{R_1 \cdot R_2}{R_1 + R_2}$) debe quedar por debajo de ~10 kΩ; valores mayores hacen que no termine de cargar el capacitor interno del ESP32.

$V_{adc} = V_{in} \cdot \frac{R_2}{R_1 + R_2}$

Para el pin de ~12,6 V con el pack cargado:

R1 = 47 kΩ; R2 = 10 kΩ:

$V_{adc} = 12,6 \cdot \frac{10}{47 + 10} \approx 2,21 \text{ V}$

$R_{th} = \frac{47 \cdot 10}{47 + 10} \approx 8,2 \text{ k}\Omega$

### Mediciones / Correcciones

Divisor resistivo: 

Vin = 11.32v, Vo = 1.995v, Ic = 0.2mA, Rth = 9.89 kohm 

Vin = 12.58v, Vo = 2.217v, 

(Factor de conversión real ≈ 5,674)

ADC en ESP32:  

- Se aplicó un capacitor de desacople de 100nF. V Multímetro = 12,55 V. Media de 20 lecturas ≈ 12,187 V. Error sistemático ≈ −0,36 V (−2,9 %).
Desviación estándar ≈ 0,098 V → ruido pico a pico de unos ±0,15 V.
- Se cambiaron las resistencias por R1= 22k y R2= 4k7
vin=12.55 vo=2.163v; Ic = 45mA; Rth = 4.61Kohm
vin=11.32 vo=1.951v
Factor ≈ 5.802
Pero bajar las resistencias no mejoró el ruido, hay un error que persiste de −0,36 a −0,59 V (≈ −3 a −5 %) puede ser un comportamiento de falta de calibración, se opta por usar una calibración por hardware (eFuse)
- Con `analogReadMilliVolts()` el error sistemático desaparece: lectura 12,46–12,54 V (±0,04 V) contra multímetro de 12,54 V. Se puede mejorar promediando muestras.
- Problema pendiente con encoders activos: aparece un offset de +0,28 V y ruido de ±0,13 V. Es ruido por la conmutación de los encoders alimentados desde el ESP32. Hay que resolverlo en hardware con capacitores de desacople 100 nF + 10 µF y/o alimentarlos desde un regulador separado. Una vez resuelto, se debe re-calibrar `BATTERY_FACTOR` con encoders + motores activos.

## Firmware en ESP32S3

Qué pines están disponibles: ADC2 lo usa el módulo WiFi, así que sólo se puede usar ADC1: GPIO 1–10.

Cambios sobre el firmware existente:

- **`config.h`**: se definieron las constantes del sensor:

```cpp
#define BATTERY_FACTOR      5.774f   // relación del divisor (Vbat / Vadc), de 22k/4k7
#define BATTERY_CORRECTION  0.9994f  // factor de calibración empírico (V_multimetro / V_leída)
#define BATTERY_SENSE       GPIO_NUM_7  // pin del ADC1
```

- **`runCommand()`**:

```cpp
case GET_BATTERY: // "b"
{
    int32_t sumMv = 0;
    for (int i = 0; i < 16; i++) sumMv += analogReadMilliVolts(BATTERY_SENSE);
    float voltage = (sumMv / 16.0f) * BATTERY_FACTOR * BATTERY_CORRECTION / 1000.0f;
    Serial.println(voltage, 2);
    break;
}
```

## Hardware Interface (`DiffDriveArduinoHardware`) con ros2_control

La lectura de batería se integró en el mismo hardware interface que ya manejaba motores y encoders.

En `arduino_comms.hpp` se agregó una función `read_battery_voltage()` que envía `'b'` y recibe como respuesta la tensión (ej. 11.5) , usa  el mismo “protocolo” request-response de los encoders. Este comando se manda cada 10 segundos

```cpp
float read_battery_voltage()
{
  std::string response = send_msg("b\r");
  return std::atof(response.c_str());
}
```

**Publicación del mensaje**

El hardware interface no puede crear publishers directamente (no es un nodo ROS 2). Se creó un nodo interno auxiliar `diffdrive_battery` en `on_configure()`, dueño de dos publishers:

- `/battery_state` — `sensor_msgs/msg/BatteryState`. El mensaje se arma con la lectura real de tensión, se calcula linealmente el porcentaje: entre `battery_voltage_min` y `battery_voltage_max`, y los campos restantes del mensaje quedan en `NaN` que es la forma estándar de indicar "este sensor no provee ese dato".
- `/battery_time_remaining` — `std_msgs/msg/Float32` con los minutos restantes estimados que se calculan como: `percentage × battery_runtime_full_min`.

Con el stack de hardware interface lanzado (controller_manager) se deberá ver: 

```bash
ros2 topic list | grep battery        # /battery_state y /battery_time_remaining
ros2 topic echo /battery_state
```

**Parámetros**

Los parámetros de hardware se declaran en `robot/description/ros2_control.xacro`, junto a los parámetros de los motores, encoders y ruedas:

```xml
<param name="battery_voltage_min">10.75</param>
<param name="battery_voltage_max">12.6</param>
<param name="battery_runtime_full_min">120.0</param>
<param name="battery_publish_period">10.0</param>
```

El código los lee al momento de inicializar.

## Interfaz Gráfica en Python con Tkinter

(captura de pantalla)

RViz2 no tiene un display nativo para `sensor_msgs/BatteryState`, por lo que se hizo una GUI para ver el estado de la batería en todo momento. Se suscribe a `/battery_state` y `/battery_time_remaining` y muestra la tensión, una barra de carga coloreada (verde >50 %, naranja 20–50 %, rojo <20 %) y minutos restantes.

Se implementó en Python con Tkinter que está incluido en la librería estándar (sin dependencias extras). La GUI corre en la PC de desarrollo y se conecta por la red DDS al resto del stack.

Ejecución:

```bash
ros2 run robot battery_monitor
```

Pruebas (sin necesidad del robot, publicando datos falsos):

```bash
ros2 topic pub /battery_state sensor_msgs/msg/BatteryState \
  '{voltage: 12.1, percentage: 0.73, power_supply_status: 2, present: true}' -r 1
ros2 topic pub /battery_time_remaining std_msgs/msg/Float32 '{data: 88.0}' -r 1
ros2 run robot battery_monitor
```
