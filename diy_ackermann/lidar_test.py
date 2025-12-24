#!/usr/bin/env python3
"""
Script de prueba para leer datos del LiDAR 2D

Uso:
    ros2 run diy_ackermann lidar_test
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np


class LidarTestNode(Node):
    def __init__(self):
        super().__init__('lidar_test')
        
        # Suscribirse al topic del LiDAR
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        self.get_logger().info('Nodo LiDAR Test iniciado!')
        self.get_logger().info('Esperando datos del LiDAR...')
    
    def scan_callback(self, msg):
        """
        Callback que se ejecuta cada vez que llega un scan del LiDAR
        """
        # Información del mensaje
        self.get_logger().info('═══════════════════════════════════════')
        self.get_logger().info(f'Timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}')
        self.get_logger().info(f'Frame ID: {msg.header.frame_id}')
        
        # Parámetros del scan
        self.get_logger().info(f'Ángulo mínimo: {msg.angle_min:.2f} rad ({np.degrees(msg.angle_min):.1f}°)')
        self.get_logger().info(f'Ángulo máximo: {msg.angle_max:.2f} rad ({np.degrees(msg.angle_max):.1f}°)')
        self.get_logger().info(f'Incremento angular: {msg.angle_increment:.4f} rad ({np.degrees(msg.angle_increment):.2f}°)')
        
        # Datos de los rangos
        ranges = np.array(msg.ranges)
        
        # Filtrar valores infinitos
        valid_ranges = ranges[np.isfinite(ranges)]
        
        if len(valid_ranges) > 0:
            self.get_logger().info(f'Total de rayos: {len(ranges)}')
            self.get_logger().info(f'Rayos válidos: {len(valid_ranges)}')
            self.get_logger().info(f'Distancia mínima detectada: {np.min(valid_ranges):.2f} m')
            self.get_logger().info(f'Distancia máxima detectada: {np.max(valid_ranges):.2f} m')
            self.get_logger().info(f'Distancia promedio: {np.mean(valid_ranges):.2f} m')
            
            # Encontrar el obstáculo más cercano
            min_idx = np.argmin(ranges)
            min_distance = ranges[min_idx]
            min_angle = msg.angle_min + min_idx * msg.angle_increment
            
            self.get_logger().info(f'Obstáculo más cercano:')
            self.get_logger().info(f'  - Distancia: {min_distance:.2f} m')
            self.get_logger().info(f'  - Ángulo: {np.degrees(min_angle):.1f}°')
            
            # Analizar por sectores (frente, izquierda, derecha, atrás)
            self.analyze_sectors(ranges, msg)
        else:
            self.get_logger().warn('No se detectaron objetos en rango')
        
        self.get_logger().info('═══════════════════════════════════════\n')
    
    def analyze_sectors(self, ranges, msg):
        """
        Analiza el scan por sectores
        """
        n = len(ranges)
        
        # Dividir en 4 sectores de 90° cada uno
        # El LiDAR tiene 360 rayos, 1 por grado
        # Índice 0 = 0°, crece antihorario
        
        # Frente: 315° a 45° (últimos 45 + primeros 45)
        front_indices = list(range(n-45, n)) + list(range(0, 45))
        front = np.array([ranges[i] for i in front_indices])
        
        # Izquierda: 45° a 135° 
        left = ranges[45:135]
        
        # Atrás: 135° a 225°
        back = ranges[135:225]
        
        # Derecha: 225° a 315°
        right = ranges[225:315]
        
        self.get_logger().info('Análisis por sectores:')
        
        for name, sector in [('Frente', back), ('Derecha', left),
                             ('Atrás', front), ('Izquierda', right)]:
            valid = sector[np.isfinite(sector)]
            if len(valid) > 0:
                min_dist = np.min(valid)
                avg_dist = np.mean(valid)
                self.get_logger().info(f'  {name:9s}: min={min_dist:.2f}m, avg={avg_dist:.2f}m')
            else:
                self.get_logger().info(f'  {name:9s}: Sin detecciones')


def main(args=None):
    rclpy.init(args=args)
    node = LidarTestNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Deteniendo nodo...')
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

