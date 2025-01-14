#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TwistReader(Node):
    def __init__(self):
        super().__init__('twist_reader')

        # Subscriber to the /ur5_control topic
        self.subscription = self.create_subscription(
            Twist,
            '/ur5_control',
            self.listener_callback,
            10
        )
        self.get_logger().info("TwistReader node initialized successfully!")

    def listener_callback(self, msg):
        # Log the received Twist message
        self.get_logger().info(f"Received Twist message:\n"
                               f"Linear: x={msg.linear.x}, y={msg.linear.y}, z={msg.linear.z}\n"
                               f"Angular: x={msg.angular.x}, y={msg.angular.y}, z={msg.angular.z}")

def main(args=None):
    rclpy.init(args=args)

    twist_reader_node = TwistReader()

    try:
        rclpy.spin(twist_reader_node)
    except KeyboardInterrupt:
        pass
    finally:
        twist_reader_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
