import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

class GoToGoal(Node):
    def __init__(self):
        super().__init__('go_to_goal')
        
        custom_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )
        
        # self.pose_subscriber = self.create_subscription(
        #     LaserScan,
        #     '/scan',
        #     self.pose_callback,
        #     qos_profile=custom_qos_profile
        # )

        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile=custom_qos_profile
        )

        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.heading = 0.0
        self.dist = 0.0
        self.laser_data = None
        self.target_points = [(0, 1.5), (1.5, 1.4), (1.5, 0)]

        self.x_robot = 0.0
        self.y_robot = 0.0
        self.current_target_index = 0

        self.init_x_robot = 0.0
        self.init_y_robot = 0.0
        self.init_heading = 0.0

        # Parameters
        self.min_distance = 0.05
        self.linear_speed = 0.1   
        self.angular_speed = 0.05
        
        self.odom_first_time = True

    def odom_callback(self, msg):
        if self.odom_first_time:
            self.get_logger().info("Yo I'm stuck in here")
            self.odom_first_time = False
            self.init_x_robot = msg.pose.pose.position.x
            self.init_y_robot = msg.pose.pose.position.y
            orientation = msg.pose.pose.orientation
            euler = self.quaternion_to_euler(orientation)
            self.init_heading = euler[2]  # yaw
            

        self.x_robot = msg.pose.pose.position.x - self.init_x_robot
        self.y_robot = msg.pose.pose.position.y - self.init_y_robot
        orientation = msg.pose.pose.orientation
        euler = self.quaternion_to_euler(orientation)
        self.heading = euler[2]  - self.init_heading # yaw
        self.get_logger().info(f'Robot Pose: x={self.x_robot:.2f}, y={self.y_robot:.2f}, theta={self.heading:.2f}')
        self.control_robot()

    def quaternion_to_euler(self, quaternion):
        # Convert quaternion to Euler angles
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = np.arctan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = np.arcsin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = np.arctan2(t3, t4)

        return (roll_x, pitch_y, yaw_z)

    def control_robot(self):
        self.get_logger().info('Controlling Robot')
        twist = Twist()
        twist.angular.z = 0.0
        twist.linear.x = self.linear_speed

        # # Get current target point
        # target_x, target_y = self.target_points[self.current_target_index]
        
        # # Calculate distance and angle to the target point
        # distance_to_target = np.sqrt((target_x - self.x_robot) ** 2 + (target_y - self.y_robot) ** 2)
        # angle_to_target = np.arctan2(target_y - self.y_robot, target_x - self.x_robot)
        # angle_error = angle_to_target - self.heading

        # # Normalize angle error
        # angle_error = (angle_error + np.pi) % (2 * np.pi) - np.pi

        # # Check if we reached the target
        # if distance_to_target <= self.min_distance:
        #     self.current_target_index += 1  # Move to the next target
        #     if self.current_target_index >= len(self.target_points):
        #         twist.linear.x = 0.0
        #         twist.angular.z = 0.0
        #         self.get_logger().info("Reached all targets!")
        #         self.cmd_vel_publisher.publish(twist)
        #         return

        # # Control logic for angular velocity
        # twist.angular.z = self.angular_speed * angle_error

        # # Control logic for linear velocity
        # twist.linear.x = self.linear_speed * (1 if distance_to_target > self.min_distance else 0)

        # self.get_logger().info(f'Moving to target: ({target_x}, {target_y}), distance={distance_to_target:.2f}, angle_error={angle_error:.2f}')

        self.cmd_vel_publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)

    go_to_goal = GoToGoal()
    rclpy.spin(go_to_goal)

    go_to_goal.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
