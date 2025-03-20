import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

class LidarObstacleDetectionNode(Node):
    def __init__(self):
        super().__init__('lidar_obstacle_detection_node')

        # Publisher for /cmd_vel to send motion commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber for LiDAR scan data
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',  # LiDAR topic name (adjust based on your setup)
            self.lidar_callback,
            10
        )

        # Obstacle detection state
        self.obstacle_distance_threshold = 0.3048  # 1 foot in meters
        self.angle_range = 10  # Degrees left and right from the front of the car
        self.obstacle_detected = False  # Initially, no obstacle is detected
        self.filter_window_size = 5  # Number of scans to average over for noise 
reduction
        self.filtered_ranges = []

    def lidar_callback(self, msg):
        # Determine the index range for the front +/- 10 degrees
        total_angles = len(msg.ranges)
        front_index = total_angles // 2  # Index of the front (0 degrees)
        start_index = max(front_index - self.angle_range, 0)
        end_index = min(front_index + self.angle_range, total_angles - 1)

        # Filter out invalid ranges (inf or NaN) in the specified angle range
        valid_ranges = [distance for distance in msg.ranges[start_index:end_index + 
1] 
                        if not (float('inf') == distance or float('nan') == 
distance)]

        # Apply a simple moving average filter to reduce noise
        self.filtered_ranges.append(valid_ranges)
        if len(self.filtered_ranges) > self.filter_window_size:
            self.filtered_ranges.pop(0)
        filtered_valid_ranges = np.mean(self.filtered_ranges, axis=0)

        # Check if any valid range in the specified angle range is below the 
threshold
        if any(distance < self.obstacle_distance_threshold for distance in 
filtered_valid_ranges):
            self.obstacle_detected = True
            self.get_logger().info("Obstacle detected within 1 foot in front: 
Swerving to avoid.")

            # Determine if the obstacle is on the left or right side
            obstacle_indices = [i for i, distance in 
enumerate(filtered_valid_ranges)
                               if distance < self.obstacle_distance_threshold]
            obstacle_position = sum(obstacle_indices) / len(obstacle_indices)  # 
Average position of obstacles

            # Calculate swerve intensity based on obstacle position
            swerve_intensity = self.calculate_swerve_intensity(obstacle_position, 
len(filtered_valid_ranges))

            # Swerve left or right based on obstacle position
            if obstacle_position < len(filtered_valid_ranges) / 2:
                self.get_logger().info("Obstacle on the left: Swerving right.")
                self.send_swerve_command(swerve_direction=1, 
intensity=swerve_intensity)  # Swerve right
            else:
                self.get_logger().info("Obstacle on the right: Swerving left.")
                self.send_swerve_command(swerve_direction=-1, 
intensity=swerve_intensity)  # Swerve left
        else:
            if self.obstacle_detected:  # Clear obstacle status if it was 
previously detected
                self.get_logger().info("Obstacle cleared: Resuming forward 
motion.")
                self.obstacle_detected = False
            self.send_move_forward_command()

    def calculate_swerve_intensity(self, obstacle_position, total_ranges):
        """
        Calculate the swerve intensity based on how close the obstacle is to the 
center.
        """
        # Normalize obstacle position to a range of [-1, 1]
        normalized_position = (obstacle_position - (total_ranges / 2)) / 
(total_ranges / 2)
        # Map to a swerve intensity (e.g., 0.1 to 0.5)
        swerve_intensity = 0.3 * abs(normalized_position)  # Adjust multiplier as 
needed
        return swerve_intensity

    def send_move_forward_command(self):
        # Create a Twist message with forward velocity
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.3  # Forward speed (adjust as needed)
        cmd_vel.angular.z = 0.0  # No angular velocity (straight motion)
        self.cmd_vel_pub.publish(cmd_vel)

    def send_swerve_command(self, swerve_direction, intensity):
        # Create a Twist message to swerve left or right
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.2  # Maintain forward speed (adjust as needed)
        cmd_vel.angular.z = -intensity * (swerve_direction)  # Swerve left or right
        self.cmd_vel_pub.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    node = LidarObstacleDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
