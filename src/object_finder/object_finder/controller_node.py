import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class ExploreController(Node):
    def __init__(self):
        super().__init__('explore_controller')
        self.sub = self.create_subscription(Bool, '/object_detected', self.detected_callback, 10)
        self.pub = self.create_publisher(Bool, '/explore/resume', 10)
        self.stopped = False

    def detected_callback(self, msg):
        """
        Called whenever the object detection node publishes on /object_detected.
        
        msg.data = True  → object has been detected in the camera frame
        msg.data = False → object is no longer visible
        
        Your task:
        - When an object IS detected: publish False on /explore/resume to PAUSE exploration
        - When an object is NOT detected: publish True on /explore/resume to RESUME exploration
        - Track whether exploration is currently stopped to avoid redundant publishes
        
        Available instance variables:
        - self.pub: publisher for /explore/resume (Bool)
        - self.stopped: bool tracking if exploration is currently paused
        """

        # =====================================================================
        # TODO [TASK 2]: Implement exploration pause/resume logic
        #
        # When msg.data is True and exploration is not already stopped:
        #   - Log that object was detected
        #   - Publish Bool(data=False) on self.pub to stop explore_lite
        #   - Set self.stopped = True
        #
        # When msg.data is False (object lost):
        #   - Log that object was lost
        #   - Publish Bool(data=True) on self.pub to resume explore_lite
        #   - Set self.stopped = False
        # =====================================================================
        
        pass  # Replace with your implementation


def main(args=None):
    rclpy.init(args=args)
    node = ExploreController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
