#!/usr/bin/env python3
"""
Solución Simple de Evasión de Obstáculos

Algoritmo básico:
- Si hay obstáculo al frente (< 1.5m): Girar izquierda
- Si no hay obstáculo: Avanzar recto

Uso:
    ros2 run diy_ackermann simple_avoider
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class SimpleAvoiderNode(Node):
    def __init__(self):
        super().__init__('simple_avoider')
        
        # Parámetros
        self.forward_speed = 0.5
        self.turn_speed = 0.5
        self.safe_distance = 0.8  # metros (aumentado para detectar antes)
        
        # Publisher para /cmd_vel
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriber para /scan
        self.scan_sub = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.scan_callback, 
            10
        )
        
        self.get_logger().info('Simple Avoider iniciado!')
        self.get_logger().info(f'Distancia segura: {self.safe_distance}m')
    
    def scan_callback(self, msg):
        """
        Procesa datos del LiDAR y toma decisión
        """
        # Obtener datos frontales
        # Usar mismo sector que en lidar_test.py (índices 135:225 = frente real)
        ranges = msg.ranges
        
        # Sector frontal (90 grados centrado al frente)
        front_ranges = ranges[135:225]
        
        # Filtrar valores infinitos y calcular mínima distancia
        valid_ranges = [r for r in front_ranges if 0.1 < r < 10.0]
        
        if len(valid_ranges) > 0:
            front_distance = min(valid_ranges)
        else:
            front_distance = 10.0  # Sin obstáculos
        
        # Tomar decisión
        if front_distance > self.safe_distance:
            # Camino despejado: avanzar
            self.move_forward()
            self.get_logger().info(f'Avanzando... (distancia: {front_distance:.2f}m)')
        else:
            # Obstáculo detectado: girar
            self.turn_left()
            self.get_logger().warn(f'¡Obstáculo! ({front_distance:.2f}m) Girando...')
    
    def move_forward(self):
        """Avanzar recto"""
        cmd = Twist()
        cmd.linear.x = self.forward_speed
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)
    
    def turn_left(self):
        """Girar a la izquierda (Ackermann necesita avanzar mientras gira)"""
        cmd = Twist()
        cmd.linear.x = self.forward_speed * 0.3  # Avanzar lento mientras gira
        cmd.angular.z = self.turn_speed
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleAvoiderNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Detener robot
        stop = Twist()
        node.cmd_pub.publish(stop)
        node.get_logger().info('Detenido')
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

