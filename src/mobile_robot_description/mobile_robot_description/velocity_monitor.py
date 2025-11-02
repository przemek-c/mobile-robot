#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class VelocityMonitor(Node):
    """ROS 2 node that monitors and prints velocity commands."""

    def __init__(self):
        super().__init__('velocity_monitor')

        # Subscribe to cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.velocity_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        self.get_logger().info('Velocity Monitor Node started. Listening to /cmd_vel topic...')

    def velocity_callback(self, msg):
        """Callback function for velocity commands."""
        self.get_logger().info(
            f'Velocity Command - Linear: x={msg.linear.x:.3f}, y={msg.linear.y:.3f}, z={msg.linear.z:.3f} | '
            f'Angular: x={msg.angular.x:.3f}, y={msg.angular.y:.3f}, z={msg.angular.z:.3f}'
        )


def main(args=None):
    rclpy.init(args=args)
    velocity_monitor = VelocityMonitor()
    rclpy.spin(velocity_monitor)

    # Clean up
    velocity_monitor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()