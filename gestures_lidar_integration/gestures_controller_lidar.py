import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class GestureController(Node):
    def __init__(self):
        super().__init__('gesture_controller')
        # Subscriber for gesture commands
        self.gesture_sub = self.create_subscription(
            String,
            'gesture',
            self.gesture_callback,
            10
        )
        # Subscriber for LiDAR scan data
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',  # LiDAR topic name (adjust based on your setup)
            self.lidar_callback,
            10
        )
        # Publisher for /cmd_vel to send motion commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Gesture control state (desired command)
        self.desired_cmd_vel = Twist()
        self.desired_cmd_vel.linear.x = 0.0  # Explicitly set to zero
        self.desired_cmd_vel.angular.z = -0.5  # Explicitly set to zero

        # Publish a zero command initially to reset any previous state
        self.publish_cmd_vel(self.desired_cmd_vel)

        # LiDAR obstacle detection state
        self.obstacle_distance_threshold = 0.5  # 0.5 meters threshold
        self.angle_range = 10  # Degrees left and right from the front of 
the car
        self.obstacle_detected = False  # Initially, no obstacle is 
detected

    def gesture_callback(self, msg):
        self.get_logger().info(f'Gesture callback triggered with data: 
{msg.data}')
        gesture = msg.data
        self.desired_cmd_vel = self.get_cmd_vel_from_gesture(gesture)
        # Publish the command only if no obstacle is detected
        if not self.obstacle_detected:
            self.publish_cmd_vel(self.desired_cmd_vel)

    def get_cmd_vel_from_gesture(self, gesture):
        cmd_vel = Twist()
        if gesture == 'OK':
            self.get_logger().info('Action: Start')
            cmd_vel.linear.x = 0.4  # Forward speed
            cmd_vel.angular.z = -0.5
        elif gesture == 'FIST':
            self.get_logger().info('Action: Stop')
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = -0.5
        elif gesture == 'ONE':
            self.get_logger().info('Action: Steer Left')
            cmd_vel.linear.x = 0.4  # Maintain forward speed
            cmd_vel.angular.z = -0.5 - 0.5  # Steer left
        elif gesture == 'PEACE':
            self.get_logger().info('Action: Steer Right')
            cmd_vel.linear.x = 0.4  # Maintain forward speed
            cmd_vel.angular.z = -0.5 + 0.5  # Steer right
        else:
            self.get_logger().info('Action: Unknown Gesture')
        return cmd_vel

    def lidar_callback(self, msg):
        total_angles = len(msg.ranges)
        front_index = total_angles // 2  # index corresponding to 0 
degrees
        start_index = max(front_index - self.angle_range, 0)
        end_index = min(front_index + self.angle_range, total_angles - 1)
        
        # Filter out invalid ranges (inf or NaN)
        valid_ranges = [distance for distance in 
msg.ranges[start_index:end_index + 1]
                        if not (distance == float('inf') or distance != 
distance)]
        
        # Decide if an obstacle is present in the forward range
        if any(distance < self.obstacle_distance_threshold for distance in 
valid_ranges):
            if not self.obstacle_detected:
                self.get_logger().info("Obstacle detected within 
threshold: Swerving to avoid.")
            self.obstacle_detected = True
            # Override with an obstacle avoidance command (swerve right)
            avoidance_cmd = Twist()
            avoidance_cmd.linear.x = 0.4  # Maintain forward speed
            avoidance_cmd.angular.z = 0.5  # Swerve right aggressively
            self.publish_cmd_vel(avoidance_cmd)
        else:
            if self.obstacle_detected:
                self.get_logger().info("Obstacle cleared: Resuming gesture 
control.")
            self.obstacle_detected = False
            # When no obstacle, use the last desired command from the 
gesture
            self.publish_cmd_vel(self.desired_cmd_vel)

    def publish_cmd_vel(self, cmd_vel):
        self.get_logger().info(f'Publishing cmd_vel: 
linear.x={cmd_vel.linear.x}, angular.z={cmd_vel.angular.z}')
        self.cmd_vel_pub.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    controller = GestureController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

