import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
import cv2
import numpy as np
import threading
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PointStamped
from tf2_geometry_msgs import tf2_geometry_msgs
from std_msgs.msg import Bool 



class ImageSubscriber1(Node):
    def __init__(self):
        super().__init__('image_subscriber1')
        
        # Create a subscriber with a queue size of 1 to only keep the last frame
        self.subscription = self.create_subscription(
            Image,
            '/camera/image',
            self.image_callback,
            1  # Queue size of 1
        )
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth_image',
            self.depth_callback,
            1
        )
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10
        )
        # base coordinates publisher
        self.object_pos_pub = self.create_publisher(
            PointStamped,
            "/Object_positon",
            10
        )
        
        self.centroid_sub = self.create_subscription(
            Point,
            "/centroid",
            self.centroid_callback,
            10
        )
        
        self.found_object_sub = self.create_subscription(
            Bool,
            "/arm_controller/found_object",
            self.found_callback,
            10
        )
        
        

        self.x = None
        self.y = None

        # Initialize CvBridge
        self.bridge = CvBridge()
        # Variable to store the latest frame
        self.latest_frame = None       
        # Variable to store the latest depth frame
        self.latest_depth_frame = None
        # Flag to control the display loop
        self.running = True 
        # Add storage for intrinsics
        self.intrinsics_ready = False
        self.fx = self.fy = self.cx = self.cy = None
        self.latest_image_stamp = None

        self.object_found = False


        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        





#   callback


    
    def found_callback(self, msg):
        self.object_found = msg.data
        self.get_logger().info("Found is Received")
    
    def centroid_callback(self, msg):
        
        # if not self.object_found:
        #     return
        self.get_logger().info("Centroid is Received")
        self.x = msg.x
        self.y = msg.y
    
    def image_callback(self, msg):
        
        # if not self.object_found:
        #     return
        
        """Callback function to receive and store the latest frame."""
        # Convert ROS Image message to OpenCV format and store it
        self.get_logger().info("image is Received")
        self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.latest_image_stamp = msg.header.stamp
    
        
    def depth_callback(self,depth_msg):
        
        # if not self.object_found:
        #     return
        
        """Callback to receive latest depth image."""
        self.latest_depth_frame = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        self.get_logger().info("depth is Received")
   
    def camera_info_callback(self, msg):
        """Extract camera intrinsics from CameraInfo message."""
        K = msg.k  # K is a flat 3x3 matrix

        self.fx = K[0]
        self.fy = K[4]
        self.cx = K[2]
        self.cy = K[5]

        self.intrinsics_ready = True
        self.get_logger().info(f"Camera intrinsics loaded: fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}")

    def display_image(self):
        
        self.get_logger().info("imaerwgwrgwtrgrtwwwv")
        
        """Main loop to process and display the latest frame."""
        # Create a single OpenCV window
        # cv2.namedWindow("frame", cv2.WINDOW_NORMAL)
        # cv2.resizeWindow("frame", 800,600)

        self.get_logger().info("fuckedv")
        
        while rclpy.ok():
            
            # self.get_logger().info("slightly less fucked")
            
            if self.object_found:
                # Check if there is a new frame available
                self.get_logger().info("Hey I am running ")
                if self.latest_frame is not None:

                    self.get_logger().info("Hey I got the latest frame")
                    # Process the current image
                    self.process_image(self.latest_frame)

                    # Show the latest frame
                    # cv2.imshow("frame", self.latest_frame)
                    self.latest_frame = None  # Clear the frame after displaying

                # Check for quit key
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self.running = False
                    break

            rclpy.spin_once(self, timeout_sec=0.05)

        # Close OpenCV window after quitting
        cv2.destroyAllWindows()
        self.running = False

    def process_image(self, img):
        if not self.intrinsics_ready:
            cv2.putText(img, "Waiting for intrinsics...", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)
            return

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        red_mask = cv2.inRange(hsv, np.array([0,120,70]), np.array([10,255,255])) | \
                cv2.inRange(hsv, np.array([170,120,70]), np.array([180,255,255]))

        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)

            if M['m00'] != 0:
                u = int(M['m10'] / M['m00'])
                v = int(M['m01'] / M['m00'])
                
                u = int(self.x)
                v = int(self.y)
            

                if self.latest_depth_frame is not None:
                    z = self.latest_depth_frame[v, u]
                    X = (u - self.cx) * z / self.fx
                    Y = (v - self.cy) * z / self.fy
                    Z = z + 0.05

                    # Create and transform point
                    pt_cam = PointStamped()
                    pt_cam.header.frame_id = 'camera_link_optical'
                    pt_cam.header.stamp = self.latest_image_stamp
                    pt_cam.point.x = X
                    pt_cam.point.y = Y
                    pt_cam.point.z = Z

                    try:
                        pt_base = self.tf_buffer.transform(
                            pt_cam, 'arm_base_link',
                            timeout=rclpy.duration.Duration(seconds=1.0)
                        )

                        # Robot frame coordinates
                        x_b = pt_base.point.x
                        y_b = pt_base.point.y
                        z_b = pt_base.point.z

                        # Create and publish the transformed point in base frame
                        msg_base = PointStamped()
                        msg_base.header.stamp = self.latest_image_stamp
                        msg_base.header.frame_id = 'arm_base_link'
                        msg_base.point.x = x_b
                        msg_base.point.y = y_b
                        msg_base.point.z = z_b

                        self.object_pos_pub.publish(msg_base)


                        # Draw point and camera coordinates
                        cv2.circle(img, (u, v), 5, (0,255,0), -1)
                        text_cam = f"(u={u}, v={v}, z={z:.2f}m)"
                        cv2.putText(img, text_cam, (u + 10, v - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,0), 1)

                        # Draw base frame coordinates in top-right
                        text_base = f"Base Frame: x={x_b:.2f}, y={y_b:.2f}, z={z_b:.2f}"
                        cv2.putText(img, text_base, (10, 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,0), 1)

                    except Exception as e:
                        self.get_logger().warn(f"TF Error: {e}")
                        cv2.putText(img, "TF transform failed", (10, 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1)



def main(args=None):

        print("OpenCV version: %s" % cv2.__version__)

        rclpy.init(args=args)
        node = ImageSubscriber1()
        
        try:
            node.display_image()  # Run the display loop
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
        main()