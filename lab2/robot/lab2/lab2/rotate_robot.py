import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist, Vector3
import numpy as np


class RotateRobot(Node):
    
    def __init__(self):
        super().__init__('rotate_robot')
        self.twist_publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.xc = 0
        self.yc = 0

        # Creating a subcription from the camera_robot_launch
        self.subscription = self.create_subscription(
            Point,
            '/object_center',
            self.point_callback,
            10)
        self.subscription    
        self.center_framex = 160.0
        self.center_framey = 120.0
    
    def point_callback(self, msg):
        self.xc = msg.x
        self.yc = msg.y
        if self.xc != -1.0 and self.yc != -1.0:
            self.get_logger().info('Calculating the twist')
            dif = self.center_framex - self.xc
            self.get_logger().info('Diff is: "%d"' % dif)
            if abs(dif) < 50:
                k = 0.05
            else:
                k = 0.02
            omega = 0.3*dif*k
            self.publish_twist(omega)
        else: 
            self.get_logger().info('No Red object found')
            self.publish_twist(0.0)
    
    def publish_twist(self, omega): 
        desired_turn = Twist()
        angular = Vector3()
        angular.x = 0.0
        angular.y = 0.0
        angular.z = omega

        linear = Vector3()
        linear.x = 0.0
        linear.y = 0.0
        linear.z = 0.0

        desired_turn.linear = linear
        desired_turn.angular = angular

        self.twist_publisher_.publish(desired_turn)
        self.get_logger().info('Publishing: "%f"' % desired_turn.angular.z)

def main(args=None):
    rclpy.init(args=args)

    rotate_robot = RotateRobot()
    rclpy.spin(rotate_robot)

    rotate_robot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()