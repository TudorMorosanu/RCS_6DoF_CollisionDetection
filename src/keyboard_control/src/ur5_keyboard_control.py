#!/usr/bin/env python3
from pynput import keyboard
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
import threading

class UR5KeyboardControl(Node):
    def __init__(self):
        super().__init__('ur5_keyboard_control')
        self.publisher_ = self.create_publisher(Twist, '/ur5_control', 10)
        self.increment_value = 0.1
        self.running = True
        self.listener_thread = threading.Thread(target=self.keyboard_listener)
        self.get_logger().info("Starting Thread...")
        self.listener_thread.start()
        self.get_logger().info("Keyboard control started. Use 'w', 's', 'a', 'd', 'q', 'e' to move the robot. Press 'esc' to quit.")

    def on_press(self, key):
        
        twist_msg = Twist()
        try:
            if key.char == 'w':
                twist_msg.linear.x = self.increment_value
            elif key.char == 's':
                twist_msg.linear.x = -self.increment_value
            elif key.char == 'a':
                twist_msg.linear.y = self.increment_value
            elif key.char == 'd':
                twist_msg.linear.y = -self.increment_value
            elif key.char == 'q':
                twist_msg.linear.z = self.increment_value
            elif key.char == 'e':
                twist_msg.linear.z = -self.increment_value
            elif key.char == 'i':
                twist_msg.angular.x = self.increment_value
            elif key.char == 'k':
                twist_msg.angular.x = -self.increment_value
            elif key.char == 'j':
                twist_msg.angular.y = self.increment_value
            elif key.char == 'l':
                twist_msg.angular.y = -self.increment_value
            elif key.char == 'u':
                twist_msg.angular.z = self.increment_value
            elif key.char == 'o':
                twist_msg.angular.z = -self.increment_value
            elif key.char == 'x':
                self.running = False
                return False

            self.get_logger().info("You pressed a key! I will publish the message.")
            self.publisher_.publish(twist_msg)
        except Exception as e:
            self.get_logger().error(f"Exception occurred: {e}")
            # Log the full traceback for debugging
            traceback_str = ''.join(traceback.format_exception(None, e, e.__traceback__))
            self.get_logger().error(f"Traceback: {traceback_str}")


    def keyboard_listener(self):
        with keyboard.Listener(on_press=self.on_press) as listener:
            listener.join()

    def destroy_node(self):
        self.running = False
        self.listener_thread.join()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = UR5KeyboardControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
