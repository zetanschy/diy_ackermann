#!/usr/bin/env python3
"""
Evasión de Obstáculos - Versión Simple Mejorada

Gira hacia el lado libre mientras avanza lento.
Cuando el camino está despejado, avanza rápido.

Uso:
    ros2 run diy_ackermann obstacle_avoidance
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np


class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        
        # Parámetros
        self.forward_speed = 0.5
        self.turn_speed = 3.0
        self.safe_distance = 1.5
        
        # Publisher y Subscriber
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        self.get_logger().info('Obstacle Avoidance (Simple) iniciado!')
    
    def scan_callback(self, msg):
        """
        Reacciona a los obstáculos en tiempo real
        """
        ranges = np.array(msg.ranges)
        
        # Analizar sectores (mismo que lidar_test)
        front = ranges[135:225]
        left = ranges[45:135]
        right = ranges[225:315]
        
        # Calcular mínimas distancias
        front_min = self.get_min_distance(front)
        left_min = self.get_min_distance(left)
        right_min = self.get_min_distance(right)
        
        cmd = Twist()
        
        if front_min > self.safe_distance:
            # Camino despejado: avanzar rápido y recto
            cmd.linear.x = self.forward_speed
            cmd.angular.z = 0.0
            self.get_logger().info(f'Avanzando... (F:{front_min:.2f}m)')
            
        else:
            # Obstáculo al frente: avanzar lento + girar hacia lado libre
            cmd.linear.x = self.forward_speed * 1.0 # Avanzar lento
            
            if left_min > right_min:
                # Girar izquierda
                cmd.angular.z = self.turn_speed
                self.get_logger().warn(
                    f'Girando IZQUIERDA (F:{front_min:.2f} L:{left_min:.2f} > R:{right_min:.2f})'
                )
            else:
                # Girar derecha
                cmd.angular.z = -self.turn_speed
                self.get_logger().warn(
                    f'Girando DERECHA (F:{front_min:.2f} R:{right_min:.2f} > L:{left_min:.2f})'
                )
        
        self.cmd_vel_pub.publish(cmd)
    
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
        node.get_logger().info('Deteniendo...')
        stop = Twist()
        node.cmd_vel_pub.publish(stop)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
