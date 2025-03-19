#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from depthai_hand_tracker.HandTracker import HandTracker
from depthai_hand_tracker.HandTrackerRenderer import HandTrackerRenderer

class GestureTracker(Node):
    def __init__(self):
        super().__init__('gesture_tracker')
        self.publisher_ = self.create_publisher(String, 'gesture', 10)
        self.hand_tracker = HandTracker(use_gesture=True)
        self.renderer = HandTrackerRenderer(self.hand_tracker)
        self.timer = self.create_timer(0.1, self.track_gesture)

    def track_gesture(self):
        frame, hands, _  = self.hand_tracker.next_frame()
        if frame is None:
            return

        if hands:
         for hand in hands:
            if hand.gesture:
                self.get_logger().info(f"Detected gesture: 
{hand.gesture}")
                msg = String()
                msg.data = hand.gesture
                self.publisher_.publish(msg)

        self.renderer.draw(frame, hands)
        self.renderer.waitKey(1)

    def destroy_node(self):
        self.hand_tracker.exit()
        self.renderer.exit()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    gesture_tracker = GestureTracker()
    try:
        rclpy.spin(gesture_tracker)
    except KeyboardInterrupt:
        pass
    finally:
        gesture_tracker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
