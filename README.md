# DIY Ackermann Robot - ROS2 Simulation Package

Paquete ROS2 para simular un robot con dirección Ackermann en Gazebo.

## Instalación

```bash
cd ~/ros2_ws/src
# Copiar o clonar el paquete aquí
cd ~/ros2_ws
colcon build --packages-select diy_ackermann
source install/setup.bash
```

## Comandos para Ejecutar las Tareas

### Lanzar Simulación

```bash
ros2 launch diy_ackermann simulation.launch.py
```

### Controlar el Robot

En otra terminal:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Tarea 2.1: Verificar LiDAR 3D

El LiDAR 3D ya está integrado. Verifica en RViz:
- Topic: `/lidar_3d/points`
- Display: PointCloud2

### Tarea 2.2: Ejecutar Movimiento Circular

```bash
ros2 run diy_ackermann circle_driver
```

O con parámetros personalizados:
```bash
ros2 run diy_ackermann circle_driver --ros-args -p radius:=3.0 -p speed:=0.8 -p direction:=1
```

### Tarea 3.1: Lanzar Circuito de Obstáculos

```bash
ros2 launch diy_ackermann simulation.launch.py world:=circuito_obstaculos.sdf
```

### Tarea 3.2: Ejecutar Evasión de Obstáculos

```bash
ros2 run diy_ackermann obstacle_avoidance
```

## Troubleshooting

### LiDAR 3D no aparece en RViz

**Problema**: El point cloud muestra "0 points from 0 messages" o no se visualiza.

**Posible Solución**: Verifica que el **Fixed Frame** en RViz esté configurado correctamente: