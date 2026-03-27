import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import cv2
from videeps_pkg.srv import SetTargetObject
import numpy as np
import threading
import math
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool 
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import tf2_ros
from tf2_geometry_msgs import do_transform_pose_stamped

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        
        # Create a subscriber with a queue size of 1 to only keep the last frame
        self.subscription = self.create_subscription(
            Image,
            'camera/image',
            self.image_callback,
            1  # Queue size of 1
        )
        
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth_image',
            self.depth_callback,
            1
        )
        
        self.point_publisher = self.create_publisher(
            Point,
            "/centroid",
            10
        )
        
        self.picked_object = self.create_subscription(
            Bool,
            "/arm_controller/is_moving",
            self.picked_object_callback,
            10
        )
        
        self.found_object_pb = self.create_publisher(
            Bool,
            "/arm_controller/found_object",
            10
        )
        
        self.picked_object_sub = self.create_subscription(
            Bool,
            "/arm_controller/picked_object",
            self.picked_callback,
            10
        )

        self.picking_object = False 
        
        self.goal_in_progress = False

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.last_goal_time = self.get_clock().now()
        self.goal_interval_sec = 2.0  # minimum interval between goals
        self.current_goal_handle = None 

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.detection_pub = self.create_publisher(Bool, "/object_detected", 10)
        self.object_found = False
        self.should_stop = False
        self.should_stop_turning = False

        self.scan_data = None
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.step_distance = 0.3
        self.error_count = 0
        self.z = float("inf")

        self.send_stop_signal = False
        
        self.latest_depth_frame = None
        
        self.stopping_distance_1 = 0.5
        self.stopping_distance_2 = 0.15
        
        # Initialize CvBridge
        self.bridge = CvBridge()

        self.Area = 320*240

        # =====================================================================
        # TODO [TASK 1]: Load your object detection model here
        # 
        # You need to:
        # 1. Load a YOLO model (or any object detection model of your choice)
        # 2. Define the list of class names your model can detect
        #
        # Example (using ultralytics):
        #   self.model = YOLO('path/to/your/model.pt')
        #   self.classNames = ["person", "bicycle", "car", ...]
        #
        # Hint: The robot needs to detect objects like "fire hydrant", "person", etc.
        # =====================================================================
        self.model = None  # TODO: Load your model
        self.classNames = []  # TODO: Define class names

        self.img_pub = self.create_publisher(Image, "/detection", 1)

        # Variable to store the latest frame
        self.latest_frame = None
        self.frame_lock = threading.Lock()  # Lock to ensure thread safety
        
        # Flag to control the display loop
        self.running = True

        # Start a separate thread for spinning (to ensure image_callback keeps receiving new frames)
        self.spin_thread = threading.Thread(target=self.spin_thread_func)
        self.spin_thread.start()
        
        # To receive the object to follow
        self.target_object = "fire hydrant"
        self.srv = self.create_service(SetTargetObject, 'set_target_object', self.set_target_callback)
        self.get_logger().info("Searching for object")
    
    def picked_callback(self, msg):
        if (msg.data == True):
            self.target_object = "person"
            self.stopping_distance_1 = 0.7
            self.stopping_distance_2 = 0.3
            self.picking_object = False     
    
    def depth_callback(self, depth_msg):
        """Callback to receive latest depth image."""
        self.latest_depth_frame = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        
    def set_target_callback(self, request, response):
        self.target_object = request.object_name
        response.success = True
        response.message = f"Now following: {self.target_object}"
        self.get_logger().info(response.message)
        return response

    def spin_thread_func(self):
        """Separate thread function for rclpy spinning."""
        while rclpy.ok() and self.running:
            rclpy.spin_once(self, timeout_sec=0.05)

    def send_incremental_goal(self, angle_rad: float, step_distance: float = 0.3):
        """
        Send a Nav2 goal that moves the robot forward by `step_distance` meters
        in the direction of `angle_rad` relative to the robot's current position.
        
        The goal is transformed from base_link frame to map frame before sending.
        """

        if self.goal_in_progress:
            return

        if not self.nav_client.wait_for_server(timeout_sec=4.0):
            self.get_logger().error("Nav2 action server not available.")
            return

        dx = math.cos(angle_rad) * self.step_distance
        dy = math.sin(angle_rad) * self.step_distance

        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = dx
        pose.pose.position.y = dy
        pose.pose.orientation.w = 1.0

        transform = self.tf_buffer.lookup_transform(
            "map", pose.header.frame_id, rclpy.time.Time())

        pose_map = do_transform_pose_stamped(pose, transform)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_map

        self.goal_in_progress = True
        self.get_logger().info(
            f"Sending nav goal to ({pose_map.pose.position.x:.2f}, {pose_map.pose.position.y:.2f})"
        )

        send_future = self.nav_client.send_goal_async(goal_msg)

        def goal_response_callback(future):
            goal_handle = future.result()
            self.current_goal_handle = goal_handle
            if not goal_handle.accepted:
                self.get_logger().warn("Goal rejected.")
                self.goal_in_progress = False
                return

            result_future = goal_handle.get_result_async()

            def result_callback(result_future):
                result = result_future.result()
                code = result.status
                if code == 4:  # ABORTED
                    self.get_logger().error("Goal aborted.")
                    self.error_count += 1
                    self.step_distance = 0.3 + self.error_count * 0.1
                    
                elif code == 3:  # SUCCEEDED
                    self.get_logger().info("Goal succeeded.")
                    self.error_count = 0
                    self.step_distance = 0.3
                else:
                    self.error_count = 0
                    self.step_distance = 0.3
                    self.get_logger().warn(f"Goal ended with status: {code}")
                    
                self.goal_in_progress = False  # Ready for next goal

            result_future.add_done_callback(result_callback)

        send_future.add_done_callback(goal_response_callback)


    def picked_object_callback(self, msg):
        self.picking_object = msg.data
    
    def image_callback(self, msg):
        """Callback function to receive and store the latest frame."""
        # Convert ROS Image message to OpenCV format and store it
        with self.frame_lock:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def scan_callback(self, msg):
        self.scan_data = msg

    def display_image(self):
        """Main loop to process and display the latest frame."""

        while rclpy.ok():
            # Check if there is a new frame available
            if self.latest_frame is not None:

                # Process the current image
                self.identify_image(self.latest_frame.copy())
                self.latest_frame = None  # Clear the frame after displaying

            # Check for quit key
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.running = False
                break

        # Close OpenCV window after quitting
        cv2.destroyAllWindows()
        self.running = False

    def identify_image(self, img):
        """
        Main detection + navigation method. Called once per camera frame.
        
        This method should:
        1. Run object detection on the image
        2. If the target object is found:
           a. Calculate its pixel centroid (cx, cy)
           b. Get depth at that pixel from self.latest_depth_frame
           c. Publish centroid to /centroid topic (Point msg with x=cx, y=cy)
           d. Publish True on /object_detected to stop exploration
           e. Navigate towards the object (use self.send_incremental_goal() or cmd_vel)
           f. When close enough (depth < self.stopping_distance_2):
              - Stop the robot
              - Publish True on /arm_controller/found_object to trigger arm pickup
        3. If the target object is NOT found in the frame:
           a. Publish False on /object_detected to resume exploration
           b. Reset detection state flags
        4. Publish the annotated/detection image to /detection topic
        
        Available instance variables:
        - self.target_object: string name of the object to find (e.g., "fire hydrant")
        - self.model: your loaded detection model
        - self.classNames: list of class names
        - self.latest_depth_frame: latest depth image (numpy array, float32, meters)
        - self.bridge: CvBridge instance
        - self.publisher: cmd_vel publisher (Twist)
        - self.detection_pub: /object_detected publisher (Bool)
        - self.point_publisher: /centroid publisher (Point)
        - self.found_object_pb: /arm_controller/found_object publisher (Bool)
        - self.img_pub: /detection image publisher (Image)
        - self.object_found: bool, whether object is currently tracked
        - self.should_stop: bool, whether robot should stop moving
        - self.picking_object: bool, whether arm is currently moving
        - self.goal_in_progress: bool, whether a Nav2 goal is active
        - self.z: float, current depth to object
        - self.stopping_distance_1: float, distance to start slowing down
        - self.stopping_distance_2: float, distance to fully stop and trigger arm
        """

        # =====================================================================
        # TODO [TASK 1]: Implement object detection and navigation logic
        #
        # Steps:
        # 1. Run your detection model on `img`
        # 2. Loop through detections, find `self.target_object`
        # 3. Calculate centroid (cx, cy) of the detected bounding box
        # 4. Read depth: self.z = self.latest_depth_frame[int(cy), int(cx)]
        # 5. Publish centroid: Point(x=cx, y=cy, z=0.0) on self.point_publisher
        # 6. If not yet found, publish True on self.detection_pub and set self.object_found = True
        # 7. Navigate toward the object:
        #    - If object is far (self.z > stopping_distance_1): send Nav2 incremental goals
        #    - If object is close (self.z < stopping_distance_1): drive straight slowly
        #    - If object is very close (self.z < stopping_distance_2): stop and signal arm
        # 8. If target not found in frame and self.object_found is True:
        #    - Reset: publish False on self.detection_pub, set self.object_found = False
        # 9. Publish annotated frame: self.img_pub.publish(self.bridge.cv2_to_imgmsg(img))
        #
        # Hint: Use self.send_incremental_goal(angle) for Nav2-based approach
        # Hint: Use self.publisher.publish(Twist(...)) for direct velocity control
        # Hint: Use self.found_object_pb.publish(Bool(data=True)) to trigger arm pickup
        # =====================================================================
        
        pass  # Replace with your implementation

    def stop(self):
        """Stop the node and the spin thread."""
        self.running = False
        self.spin_thread.join()

def main(args=None):

    print("OpenCV version: %s" % cv2.__version__)

    rclpy.init(args=args)
    node = ImageSubscriber()
    
    try:
        node.display_image()  # Run the display loop
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()