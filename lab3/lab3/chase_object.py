import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
import numpy as np
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

class ObjectFollower(Node):
    def __init__(self):
        super().__init__('object_follower')
        
        custom_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )
        
        self.pose_subscriber = self.create_subscription(
            Pose2D,
            '/robot_to_object',
            self.pose_callback,
            qos_profile=custom_qos_profile
        )

        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.heading = 0.0
        self.dist = 0.0
        self.laser_data = None

        # Parameters
        self.min_distance = 0.4
        self.target_angle = 0
        self.linear_speed = 0.2   
        self.angular_speed = 0.05

    def pose_callback(self, msg):
        # self.get_logger().info(f'Received object pose: x={msg.x}, y={msg.y}, theta={msg.theta}')
        self.heading = msg.theta
        self.dist = msg.x
        self.control_robot()

    def control_robot(self):
        if self.heading == 0:
            return
        
        twist = Twist()

        # obj_x = self.object_pose.x
        # obj_y = self.object_pose.y
        # distance_to_object = np.sqrt(obj_x ** 2 + obj_y ** 2)
        # dist_error = self.min_distance - distance_to_object
        
        # obj_theta = self.object_pose.theta
        # angle_error = self.target_angle - obj_theta
        dist_error = self.dist - self.min_distance
        angle_error = self.heading
        if abs(angle_error) < 50:
            k = 0.05
        else:
            k = 0.02
        twist.angular.z = 0.3*angle_error*k

        if abs(dist_error) > 0.2:
            alpha = 1
        else:
            alpha = 5
        twist.linear.x = self.linear_speed*alpha*dist_error



        self.get_logger().info(f'Moving towards object: distance error ={dist_error:.2f}, angle={self.heading:.2f}')

        self.cmd_vel_publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)

    object_follower = ObjectFollower()
    rclpy.spin(object_follower)

    object_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
