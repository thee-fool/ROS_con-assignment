import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool
import math
import time

class JointAnglePublisher(Node):
    def __init__(self):
        super().__init__('joint_angle_publisher')

        self.publisher = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )

        self.armMovingpb = self.create_publisher(
            Bool,
            "/arm_controller/is_moving",
            10)
        
        self.subscription = self.create_subscription(
            PointStamped,
            '/Object_positon',
            self.object_callback,
            10
        )
        
        self.found_object_pb = self.create_publisher(
            Bool,
            "/arm_controller/found_object",
            10
        )
        
        self.picked_object_pb = self.create_publisher(
            Bool,
            "/arm_controller/picked_object",
            10
        )

        self.get_logger().info('JointAnglePublisher ready and listening to /Object_positon')

        self.has_moved = False
        self.is_moving = False
        self.timer = self.create_timer(0.1, self.my_function)
        
    def my_function(self):
        lol = Bool()
        lol.data = self.is_moving
        self.armMovingpb.publish(lol)

    def object_callback(self, msg: PointStamped):
        
        if self.has_moved:
            self.get_logger().info("Motion already executed. Ignoring new object position.")
            return
        self.is_moving = True 
        x = msg.point.x
        y = msg.point.y
        z = msg.point.z
        self.get_logger().info(f"Received object at: x={x:.2f}, y={y:.2f}, z={z:.2f}")

        self.publish_trajectory(x, y, z + 0.15, "open", 10)   # Approach from top
        self.publish_trajectory(x-0.03, y, z-0.03, "open", 5)          # Move straight down
        self.publish_trajectory(x-0.03, y, z-0.03, "closed", 2)        # Close gripper
        self.publish_trajectory(x, y, z + 0.21, "closed", 3) # Lift up
        self.publish_trajectory(x-0.2, y, z + 0.25, "closed", 3) # move back

        self.get_logger().info("Pick-up sequence complete.")
        
        self.is_moving = False
        self.has_moved = True
        lmao = Bool()
        lmao.data = False
        
        picked = Bool()
        picked.data = self.has_moved
        
        self.found_object_pb.publish(lmao)
        self.picked_object_pb.publish(picked)

    def publish_trajectory(self, x, y, z, gripper_status, delay_sec=2):
        joint_angles = self.inverse_kinematics([x, y, z], gripper_status=gripper_status, gripper_angle=0)

        traj = JointTrajectory()
        traj.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_joint', 'left_finger_joint', 'right_finger_joint'
        ]

        point = JointTrajectoryPoint()
        point.positions = joint_angles
        point.velocities = [0.0] * 6
        point.time_from_start.sec = delay_sec
        traj.points = [point]

        self.publisher.publish(traj)
        self.get_logger().info(f"Published joint trajectory with gripper: {gripper_status}")
        time.sleep(delay_sec + 1)  # Allow motion to complete before next

    def inverse_kinematics(self, coords, gripper_status, gripper_angle=0):
        ua_link = 0.2
        fa_link = 0.25
        tcp_link = 0.175
        z_offset = 0.05

        angles = [0, 0, 0, 0, 0, 0]

        j0 = math.atan2(coords[1], coords[0])

        x = coords[0] - tcp_link * math.cos(j0) * math.cos(gripper_angle)
        y = coords[1] - tcp_link * math.sin(j0) * math.cos(gripper_angle)
        z = coords[2] - z_offset + math.sin(gripper_angle) * tcp_link

        x = math.sqrt(x ** 2 + y ** 2)
        c = math.sqrt(x * x + z * z)

        try:
            alpha = math.asin(z / c)
            gamma = math.acos((ua_link ** 2 + c ** 2 - fa_link ** 2) / (2 * ua_link * c))
            beta = math.pi - alpha
            j1 = math.pi / 2 - alpha - gamma
            j2 = math.pi - math.acos((ua_link ** 2 + fa_link ** 2 - c ** 2) / (2 * ua_link * fa_link))
            delta = math.pi - (math.pi - j2) - gamma
            j3 = math.pi + gripper_angle - beta - delta
        except ValueError:
            self.get_logger().error("Target out of reach. IK math domain error.")
            return angles

        angles[0] = j0
        angles[1] = j1
        angles[2] = j2
        angles[3] = j3

        if gripper_status == "open":
            angles[4] = 0.04
            angles[5] = 0.04
        else:
            angles[4] = 0.01
            angles[5] = 0.01

        return angles

def main(args=None):
    rclpy.init(args=args)
    node = JointAnglePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
