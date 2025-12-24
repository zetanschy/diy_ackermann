#!/usr/bin/env python3
"""
Go To Goal Controller for Ackermann Robot

Controla el robot diy_ackermann para ir a una posición objetivo (x, y).

Uso:
    ros2 run diy_ackermann go_to_goal --ros-args -p goal_x:=3.0 -p goal_y:=2.0
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def euler_from_quaternion(q):
    """Convierte quaternion a euler (roll, pitch, yaw)."""
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return roll, pitch, yaw


def normalize_angle(angle):
    """Normaliza ángulo a [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class GoToGoal(Node):
    def __init__(self):
        super().__init__('go_to_goal')
        
        # Parámetros del robot (diy_ackermann)
        self.wheelbase = 0.4  # metros
        self.max_steering = 0.5236  # 30 grados
        
        # Parámetros del controlador
        self.declare_parameter('goal_x', 2.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('linear_speed', 0.5)
        self.declare_parameter('kp_angular', 1.5)
        self.declare_parameter('distance_tolerance', 0.15)
        
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.kp_angular = self.get_parameter('kp_angular').value
        self.distance_tolerance = self.get_parameter('distance_tolerance').value
        
        # Estado actual
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.goal_reached = False
        
        # Publishers y Subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Timer de control (20 Hz)
        self.timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info(f'Go To Goal iniciado')
        self.get_logger().info(f'Objetivo: ({self.goal_x:.2f}, {self.goal_y:.2f})')
    
    def odom_callback(self, msg):
        """Actualiza la pose del robot desde odometría."""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        _, _, self.theta = euler_from_quaternion(msg.pose.pose.orientation)
    
    def control_loop(self):
        """Loop principal de control."""
        if self.goal_reached:
            return
        
        # Calcular error de distancia
        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Verificar si llegamos
        if distance < self.distance_tolerance:
            self.stop_robot()
            self.goal_reached = True
            self.get_logger().info(f'Goal alcanzado! Posición final: ({self.x:.2f}, {self.y:.2f})')
            return
        
        # Calcular ángulo hacia el objetivo
        angle_to_goal = math.atan2(dy, dx)
        
        # Error angular
        angle_error = normalize_angle(angle_to_goal - self.theta)
        
        # Controlador P para velocidad angular
        omega = self.kp_angular * angle_error
        
        # Limitar omega basado en la velocidad lineal y steering máximo
        # δ = atan(L * ω / v) => ω_max = v * tan(δ_max) / L
        if self.linear_speed > 0.01:
            omega_max = self.linear_speed * math.tan(self.max_steering) / self.wheelbase
            omega = max(-omega_max, min(omega_max, omega))
        
        # Reducir velocidad si el error angular es grande
        if abs(angle_error) > math.pi / 4:  # > 45 grados
            v = self.linear_speed * 0.3
        elif abs(angle_error) > math.pi / 8:  # > 22.5 grados
            v = self.linear_speed * 0.6
        else:
            v = self.linear_speed
        
        # Reducir velocidad cerca del objetivo
        if distance < 0.5:
            v = v * (distance / 0.5)
            v = max(v, 0.1)  # velocidad mínima
        
        # Publicar comando
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = omega
        self.cmd_pub.publish(cmd)
        
        # Log periódico
        self.get_logger().info(
            f'Pos: ({self.x:.2f}, {self.y:.2f}) | '
            f'Dist: {distance:.2f}m | '
            f'Error ang: {math.degrees(angle_error):.1f}°',
            throttle_duration_sec=0.5
        )
    
    def stop_robot(self):
        """Detiene el robot."""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = GoToGoal()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrumpido por usuario')
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

