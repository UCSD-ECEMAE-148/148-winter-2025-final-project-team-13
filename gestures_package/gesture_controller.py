#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class GestureController(Node):
    def __init__(self):
        super().__init__('gesture_controller')
        self.subscription = self.create_subscription(
            String,
            'gesture',
            self.gesture_callback,
            10
        )

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.moving = False

    def gesture_callback(self, msg):
        gesture = msg.data
        self.get_logger().info(f'Received gesture: {gesture}')
        self.perform_action_based_on_gesture(gesture)

    def perform_action_based_on_gesture(self, gesture):

        cmd_vel = Twist()

        if gesture == 'OK':
            self.get_logger().info('Action: Start')
            cmd_vel.linear.x = 0.5  # Increase speed to ensure movement
            cmd_vel.angular.z = 0.0  # Make it move straight ahead
            self.moving = True

        elif gesture == 'FIST':
            self.get_logger().info('Action: Stop')
            cmd_vel.linear.x = 0.0  # Stop the car
            cmd_vel.angular.z = 0.0
            self.moving = False

        elif gesture == 'ONE':
            self.get_logger().info('Action: Steer Left')
            cmd_vel.linear.x = 0.5
            cmd_vel.angular.z = 0.5  # Steer left

        elif gesture == 'PEACE':
            self.get_logger().info('Action: Steer Right')
            cmd_vel.linear.x = 0.5
            cmd_vel.angular.z = -0.5  # Steer right
    
        else:
            self.get_logger().info('Action: Unknown Gesture')

        # Publish the command
        self.cmd_vel_pub.publish(cmd_vel)


def main(args=None):
    rclpy.init(args=args)
    gesture_controller = GestureController()
    try:
        rclpy.spin(gesture_controller)
    except KeyboardInterrupt:
        pass
    finally:
        gesture_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
