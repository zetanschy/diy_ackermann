# Tarea para Casa - DIY Ackermann Robot

## Parte 1: ACTIVIDAD 1 - LiDAR 3D + Movimiento Circular

### Objetivo

Añadir un sensor LiDAR 3D al robot e implementar un controlador para movimiento en círculo. Documentar con video/GIF.

### Tarea 2.1: Integrar LiDAR 3D

Añade un **LiDAR 3D** (tipo Velodyne) al robot.

**Especificaciones del LiDAR 3D:**
- Tipo: `gpu_lidar` (si tienes GPU dedicada) o `lidar` (CPU)
- Canales verticales: 16
- Resolución horizontal: 1800 samples
- Rango: 0.5m - 30m
- Frecuencia: 10 Hz
- Ángulo vertical: -15° a +15°
- Posición: Arriba del chasis, centrado

> **Nota sobre GPU:** Si tu computadora tiene GPU dedicada (NVIDIA/AMD), usa `type="gpu_lidar"` para mejor rendimiento. Si no tienes GPU o el simulador es lento, usa `type="lidar"`. Ambos funcionan igual, solo cambia el rendimiento.

**Código de referencia:**

```xml
<!-- 1. Propiedades del sensor -->
<xacro:property name="lidar3d_size" value="0.15"/>
<xacro:property name="lidar3d_mass" value="0.8"/>

<!-- 2. Link del LiDAR 3D -->
<link name="lidar_3d_link">
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <cylinder radius="${lidar3d_size/2}" length="${lidar3d_size}"/>
    </geometry>
    <material name="blue"/>
  </visual>
  
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <cylinder radius="${lidar3d_size/2}" length="${lidar3d_size}"/>
    </geometry>
  </collision>
  
  <xacro:cylinder_inertia mass="${lidar3d_mass}" 
                          radius="${lidar3d_size/2}" 
                          height="${lidar3d_size}"/>
</link>

<!-- 3. Joint -->
<joint name="lidar_3d_joint" type="fixed">
  <origin xyz="0 0 ${chassis_height/2 + lidar3d_size + 0.05}" rpy="0 0 0"/>
  <parent link="chassis"/>
  <child link="lidar_3d_link"/>
</joint>

<!-- 4. Sensor Gazebo -->
<gazebo reference="lidar_3d_link">
  <sensor name="lidar_3d" type="gpu_lidar">  <!-- Cambiar a type="lidar" si no tienes GPU -->
    <topic>lidar_3d/points</topic>
    <update_rate>10</update_rate>
    <lidar>
      <scan>
        <horizontal>
          <samples>1800</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
        <vertical>
          <samples>16</samples>
          <resolution>1</resolution>
          <min_angle>-0.2617</min_angle>  <!-- -15° -->
          <max_angle>0.2617</max_angle>   <!-- +15° -->
        </vertical>
      </scan>
      <range>
        <min>0.5</min>
        <max>30.0</max>
        <resolution>0.01</resolution>
      </range>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.02</stddev>
      </noise>
    </lidar>
    <always_on>1</always_on>
    <visualize>true</visualize>
  </sensor>
</gazebo>

<!-- 5. Material Gazebo -->
<gazebo reference="lidar_3d_link">
  <material>Gazebo/Blue</material>
</gazebo>
```

**Bridge para PointCloud2:**
```python
# En simulation.launch.py añadir:
bridge_lidar_3d = Node(
    package="ros_gz_bridge",
    executable="parameter_bridge",
    name="bridge_lidar_3d",
    arguments=[
        "/lidar_3d/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked"
    ],
    output="screen"
)
```

### Tarea 2.2: Implementar Movimiento Circular

Crea un nodo que haga que el robot conduzca en un patrón circular continuo.

**Especificaciones:**
- Radio del círculo: Configurable por parámetro (default: 2.0m)
- Velocidad lineal: Configurable por parámetro (default: 0.5 m/s)
- Dirección: Configurable (1 = izquierda, -1 = derecha)
- Movimiento circular continuo
- Calcular automáticamente velocidad angular: ω = v/r
- Debe imprimir información del movimiento en consola

**Plantilla del código:**

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CircleDriverNode(Node):
    def __init__(self):
        super().__init__('circle_driver')
        
        # TODO: Declarar parámetros
        # - radius (float, default: 2.0)
        # - speed (float, default: 0.5)
        # - direction (int, default: 1)
        
        # TODO: Obtener valores de los parámetros
        # self.radius = ...
        # self.speed = ...
        # self.direction = ...
        
        # TODO: Calcular velocidad angular usando la fórmula ω = v/r
        # Hint: self.angular_speed = ...
        
        # TODO: Crear publisher para /cmd_vel
        # self.cmd_vel_pub = ...
        
        # TODO: Crear timer que llame a control_loop cada 0.1 segundos
        # self.timer = ...
        
        # Log de información
        self.get_logger().info('Circle Driver iniciado!')
        # TODO: Añadir más información útil (radio, velocidad, omega)
        
    def control_loop(self):
        """
        Esta función se llama cada 0.1 segundos
        Debe publicar velocidades para mantener movimiento circular
        """
        # TODO: Crear mensaje Twist
        
        # TODO: Asignar velocidad lineal
        # cmd.linear.x = ...
        
        # TODO: Asignar velocidad angular
        # cmd.angular.z = ...
        
        # TODO: Publicar el comando
        pass

def main(args=None):
    rclpy.init(args=args)
    node = CircleDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # TODO: Detener el robot publicando velocidades en cero
        node.get_logger().info('Detenido')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Hints importantes:**
- La fórmula clave es: `ω = v / r`
- Para publicar en `/cmd_vel` usa `geometry_msgs/msg/Twist`
- `Twist.linear.x` es la velocidad hacia adelante
- `Twist.angular.z` es la velocidad de rotación (positivo = izquierda)
- Usa `self.create_timer(tiempo, callback)` para ejecutar código periódicamente

---

### Tarea 2.3: Video/GIF Demostrativo

---

## Parte 2: ACTIVIDAD 2 - Circuito de Obstáculos + Evasión

### Objetivo

Implementar un algoritmo de evasión de obstáculos y probar navegación autónoma en un circuito predefinido.

### Tarea 3.1: Circuito de Prueba Predefinido

**NO necesitas crear el mundo**, usa el mundo `circuito_obstaculos.sdf` ya incluido.

**Descripción del circuito:**
- Pasillo inicial (10m) con paredes laterales
- Obstáculos estratégicamente ubicados:
  - Caja en centro del pasillo (posición x=3, y=0)
  - Dos cilindros flanqueando (x=5, y=±1.5)
  - Zona con pasos estrechos
  - Área abierta con obstáculos dispersos
  - Pared final como meta

**Para lanzar el circuito:**
```bash
ros2 launch diy_ackermann simulation.launch.py world:=circuito_obstaculos.sdf
```

**Mapa del circuito:**
```
Inicio (0,0)
    │
    ├─── Obstáculo 1: Caja central (3, 0)
    │
    ├─── Obstáculos 2-3: Cilindros laterales (5, ±1.5)
    │
    ├─── Obstáculo 4: Paso estrecho (8, -2)
    │
    ├─── Obstáculo 5: Esquina (10, 2)
    │
    └─── Meta: Pared verde (12, 0)
```

### Tarea 3.2: Implementar Evasión de Obstáculos

Crea un nodo que detecte obstáculos y gire hacia el lado libre para evitarlos.

**Requisitos:**
1. Suscribirse al topic `/scan` del LiDAR 2D
2. Analizar 3 sectores: Frente, Izquierda, Derecha
3. Si NO hay obstáculo al frente: avanzar rápido
4. Si HAY obstáculo al frente: girar hacia el lado con más espacio libre

> **Nota:** El robot debe girar hacia el lado que tenga más espacio (comparar izquierda vs derecha).

**Algoritmo:**

```
Analizar 3 sectores del LiDAR:
  - Frente: índices 135:225
  - Izquierda: índices 45:135  
  - Derecha: índices 225:315

SI frente > 1.5m:
    Avanzar rápido: linear.x = 0.5, angular.z = 0.0
SINO:
    Avanzar lento: linear.x = 0.3
    SI izquierda > derecha:
        Girar izquierda: angular.z = +1.0
    SINO:
        Girar derecha: angular.z = -1.0
```

**Plantilla del código:**

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        
        # Parámetros
        self.forward_speed = 0.5
        self.turn_speed = 1.0
        self.safe_distance = 1.5  # metros
        
        # TODO: Crear publisher para /cmd_vel
        # self.cmd_pub = self.create_publisher(...)
        
        # TODO: Crear subscriber para /scan
        # self.scan_sub = self.create_subscription(...)
        
        self.get_logger().info('Obstacle Avoidance iniciado!')
    
    def scan_callback(self, msg):
        """
        Analiza LiDAR y decide hacia dónde girar
        """
        ranges = np.array(msg.ranges)
        
        # TODO: Definir sectores (usar índices correctos)
        # front = ranges[135:225]
        # left = ranges[45:135]
        # right = ranges[225:315]
        
        # TODO: Calcular distancias mínimas
        # front_min = self.get_min_distance(front)
        # left_min = self.get_min_distance(left)
        # right_min = self.get_min_distance(right)
        
        # TODO: Crear comando Twist()
        
        # TODO: Decidir acción
        # SI front_min > self.safe_distance:
        #     Avanzar rápido recto
        # SINO:
        #     Avanzar lento + girar hacia lado con más espacio
        #     SI left_min > right_min:
        #         angular.z = +self.turn_speed  (izquierda)
        #     SINO:
        #         angular.z = -self.turn_speed  (derecha)
        
        # TODO: Publicar comando
        pass
    
    def get_min_distance(self, sector):
        """Obtiene distancia mínima filtrando infinitos"""
        valid = sector[np.isfinite(sector)]
        return np.min(valid) if len(valid) > 0 else 10.0

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        stop = Twist()
        node.cmd_pub.publish(stop)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Archivo:** `diy_ackermann/obstacle_avoidance.py`

**Registrar en setup.py:**
```python
entry_points={
    'console_scripts': [
        'lidar_test = diy_ackermann.lidar_test:main',
        'circle_driver = diy_ackermann.circle_driver:main',
        'obstacle_avoidance = diy_ackermann.obstacle_avoidance:main',
    ],
},
```

### Tarea 3.3: Video Demostrativo del Circuito
