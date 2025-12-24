#!/usr/bin/env python3
"""
Simple Circle Driver - Example controller for Ackermann Robot

This node makes the robot drive in a circle by publishing constant
velocity commands to /cmd_vel.

Educational purpose: Shows how to create a basic ROS2 node that controls the robot.

Usage:
    ros2 run ackermann_robot circle_driver

    OR with custom parameters:
    ros2 run ackermann_robot circle_driver --ros-args -p linear_velocity:=1.0 -p angular_velocity:=0.5
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CircleDriverNode(Node):
    """
    A simple node that makes the Ackermann robot drive in a circle.
    """

    def __init__(self):
        super().__init__('circle_driver')
        
        # Declare parameters with default values
        self.declare_parameter('linear_velocity', 0.5)  # m/s forward speed
        self.declare_parameter('angular_velocity', 0.3)  # rad/s turning rate
        self.declare_parameter('publish_rate', 10.0)    # Hz
        
        # Get parameter values
        self.linear_vel = self.get_parameter('linear_velocity').value
        self.angular_vel = self.get_parameter('angular_velocity').value
        publish_rate = self.get_parameter('publish_rate').value
        
        # Create publisher for velocity commands
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # Create timer to publish commands at regular intervals
        timer_period = 1.0 / publish_rate  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info(
            f'Circle Driver started! '
            f'Linear: {self.linear_vel} m/s, '
            f'Angular: {self.angular_vel} rad/s'
        )
        self.get_logger().info('Press Ctrl+C to stop')

    def timer_callback(self):
        """
        Called periodically by the timer. Publishes velocity commands.
        """
        # Create a Twist message
        msg = Twist()
        
        # Set linear velocity (forward/backward)
        msg.linear.x = self.linear_vel  # Forward
        msg.linear.y = 0.0              # No sideways (Ackermann can't do this)
        msg.linear.z = 0.0              # No up/down
        
        # Set angular velocity (turning)
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = self.angular_vel  # Turning left/right
        
        # Publish the message
        self.cmd_vel_publisher.publish(msg)


def main(args=None):
    """
    Main function - entry point for the node.
    """
    # Initialize ROS2
    rclpy.init(args=args)
    
    # Create the node
    node = CircleDriverNode()
    
    try:
        # Keep the node running until Ctrl+C
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Ctrl+C pressed - stop gracefully
        node.get_logger().info('Stopping robot...')
        
        # Send stop command
        stop_msg = Twist()  # All zeros = stop
        node.cmd_vel_publisher.publish(stop_msg)
    
    finally:
        # Cleanup
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

