import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy


class Obstacle_Avoidance(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        custom_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )
        self._publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self._subscriber = self.create_subscription(LaserScan, "/scan", self.laser_scan_process, qos_profile=custom_qos_profile)
        self._velocity_msg = Twist()
        self._move_forward_velocity = 0.0
        self._rotate_angle_velocity = 0.0

    def laser_scan_process(self, msg):
        message_range = msg.ranges
        field_range = 60
        initial_angle = 330
        obstacle_detected = False

        # Make sure initial_angle + field_range does not exceed the length of message_range
        for i in range(initial_angle, initial_angle + field_range):
            index = i % 360  # Wrap around using modulo
            if index < len(message_range) and message_range[index] < 0.75:
                obstacle_detected = True
                break
        
        if obstacle_detected:
            self.get_logger().info('Obstacle detected')
            self.publish_action(angular=-0.4,linear=0.0)
        else:
            self.get_logger().info('Just going straightttt')
            self.publish_action(angular=0.0,linear=0.3)

    
    def publish_action(self,angular,linear):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self._publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)  
    node = Obstacle_Avoidance("object_avoidance") 
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Log a message when the node is manually terminated
        node.get_logger().warn("Keyboard interrupt detected")
    finally:
        # Cleanly destroy the node instance
        node.destroy_node()
        # Shut down the ROS 2 Python client library
        rclpy.shutdown()


if __name__ == "__main__":
    main()  

