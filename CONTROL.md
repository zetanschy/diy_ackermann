# Control del Robot Ackermann

## Parámetros del Robot

| Parámetro | Valor |
|-----------|-------|
| Wheelbase (L) | 0.4 m |
| Track Width | 0.5 m |
| Wheel Radius | 0.15 m |
| Max Steering | 30° (0.5236 rad) |

## Topics

| Topic | Tipo | Dirección |
|-------|------|-----------|
| `/cmd_vel` | `geometry_msgs/Twist` | Entrada |
| `/odom` | `nav_msgs/Odometry` | Salida |
| `/scan` | `sensor_msgs/LaserScan` | Salida |

## Modelo Cinemático Ackermann

```
        (Front)
           |
           | L (wheelbase)
           |
    ───────┼───────
           |
           δ = atan(L * ω / v)
```

**Relación steering - velocidad angular:**
```
δ = atan(L * ω / v)
```

Donde:
- `δ` = ángulo de steering
- `L` = wheelbase (0.4m)
- `ω` = velocidad angular deseada
- `v` = velocidad lineal

## Lanzar Simulación

```bash
# Terminal 1: Gazebo
ros2 launch diy_ackermann simulation.launch.py

# Terminal 2: RViz
ros2 launch diy_ackermann rviz.launch.py
```

## Ejecutar Go To Goal

```bash
# Terminal 3: Controlador
ros2 run diy_ackermann go_to_goal --ros-args -p goal_x:=3.0 -p goal_y:=2.0
```

### Parámetros del Controlador

| Parámetro | Default | Descripción |
|-----------|---------|-------------|
| `goal_x` | 2.0 | Posición X objetivo (metros) |
| `goal_y` | 0.0 | Posición Y objetivo (metros) |
| `linear_speed` | 0.5 | Velocidad lineal máxima (m/s) |
| `kp_angular` | 1.5 | Ganancia proporcional angular |
| `distance_tolerance` | 0.15 | Distancia para considerar goal alcanzado |

---

## Explicación del Script `go_to_goal.py`

### Diagrama del Algoritmo

```
┌─────────────────────────────────────────────────────────┐
│                    LOOP DE CONTROL                       │
└─────────────────────────────────────────────────────────┘
                          │
                          ▼
              ┌───────────────────────┐
              │  Leer odometría       │
              │  (x, y, θ)            │
              └───────────────────────┘
                          │
                          ▼
              ┌───────────────────────┐
              │  Calcular distancia   │
              │  d = √(dx² + dy²)     │
              └───────────────────────┘
                          │
                          ▼
              ┌───────────────────────┐
              │  ¿d < tolerancia?     │──── Sí ───▶ STOP
              └───────────────────────┘
                          │ No
                          ▼
              ┌───────────────────────┐
              │  Calcular ángulo      │
              │  hacia el goal        │
              │  α = atan2(dy, dx)    │
              └───────────────────────┘
                          │
                          ▼
              ┌───────────────────────┐
              │  Calcular error       │
              │  angular              │
              │  e = α - θ            │
              └───────────────────────┘
                          │
                          ▼
              ┌───────────────────────┐
              │  Control P            │
              │  ω = Kp * e           │
              └───────────────────────┘
                          │
                          ▼
              ┌───────────────────────┐
              │  Limitar ω según      │
              │  steering máximo      │
              └───────────────────────┘
                          │
                          ▼
              ┌───────────────────────┐
              │  Ajustar v según      │
              │  error angular        │
              └───────────────────────┘
                          │
                          ▼
              ┌───────────────────────┐
              │  Publicar cmd_vel     │
              │  (v, ω)               │
              └───────────────────────┘
```

### Partes Principales del Código

**1. Cálculo del error de posición:**
```python
dx = goal_x - x
dy = goal_y - y
distance = sqrt(dx² + dy²)
```

**2. Cálculo del ángulo hacia el objetivo:**
```python
angle_to_goal = atan2(dy, dx)
angle_error = angle_to_goal - theta  # normalizado a [-π, π]
```

**3. Control proporcional para ω:**
```python
omega = Kp * angle_error
```

**4. Limitación por steering máximo:**

El robot Ackermann no puede girar en el lugar. La velocidad angular está limitada por:
```python
omega_max = v * tan(δ_max) / L

# Donde:
# δ_max = 0.5236 rad (30°)
# L = 0.4 m (wheelbase)
```

**5. Ajuste de velocidad lineal:**

- Si error angular > 45°: reducir a 30% de velocidad
- Si error angular > 22.5°: reducir a 60% de velocidad
- Si distancia < 0.5m: reducir proporcionalmente

### Limitaciones del Ackermann

A diferencia de un robot diferencial, el Ackermann:
- NO puede girar en el lugar (radio de giro mínimo)
- Debe avanzar para poder girar
- El radio de giro depende del steering: `R = L / tan(δ)`

Por eso el script reduce la velocidad cuando el error angular es grande, permitiendo que el robot haga curvas más cerradas mientras avanza lentamente.

---

## Comandos Útiles

```bash
# Ver odometría
ros2 topic echo /odom

# Enviar velocidad manual
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"

# Detener
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

