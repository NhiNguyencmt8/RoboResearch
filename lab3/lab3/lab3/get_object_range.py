import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose2D, Vector3
from sensor_msgs.msg import LaserScan
import numpy as np
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

class GetObjectRange(Node):
    
    def __init__(self):
        super().__init__('get_object_range')
        custom_qos_profile = QoSProfile(
            reliability = QoSReliabilityPolicy .BEST_EFFORT,
            history = QoSHistoryPolicy .KEEP_LAST,
            durability = QoSDurabilityPolicy .VOLATILE,
            depth = 1
        )

        self.xo = 0
        self.yo = 0
        self.theta = 0
        self.dist = 0
        self.xc = 0
        self.yc = 0

        # Sub from /scan
        self.scan = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_distance_callback,
            qos_profile=custom_qos_profile)
        
        self.scan

        # Sub from /object_center

        self.center = self.create_subscription(
            Point,
            '/object_center',
            self.point_callback,
            10)
        self.center 

        # Object pose
    
        self.pose_publisher_ = self.create_publisher(Pose2D, '/robot_to_object', 10)
    
    def scan_distance_callback(self, msg):
        #Code inspired by https://github.com/ROBOTIS-GIT/turtlebot3/blob/master/turtlebot3_example/nodes/turtlebot3_obstacle
        scan_filter = []
        samples_view = 1

        if samples_view == 1:
            scan_filter.append(msg.ranges[0])
        else:
            left_lidar_samples_ranges = -(samples_view // 2 + samples_view % 2)
            right_lidar_samples_ranges = samples_view // 2

            left_lidar_samples = msg.ranges[left_lidar_samples_ranges:]
            right_lidar_samples = msg.ranges[:right_lidar_samples_ranges]
            scan_filter.extend(left_lidar_samples + right_lidar_samples)

        for i in range(samples_view):
            if scan_filter[i] == float('Inf'):
                scan_filter[i] = 3.5
            elif np.isnan(scan_filter[i]):
                scan_filter[i] = 0

        dist = min(scan_filter) # Distance is in m
        self.get_logger().info('Scan distance is ' + str(dist))
        self.dist = dist   
        return dist
    
    def point_callback(self, msg):
        if msg.x != -1.0 and msg.y != -1.0:
            self.xc = msg.x
            self.yc = msg.y
            self.get_logger().info('Calculating pose')
            self.pose_estimate()
        else: 
            self.get_logger().info('No Red object found')
    
    def pose_estimate(self):
        final = Pose2D()

        # Focal length in pixels
        fx = 640 / (2*np.tan((62.2*np.pi/180)/2)) # Convert deg to rad and then calculate the horizontal field of view in radians
        fy = 480 / (2*np.tan((48.8*np.pi/180)/2))

        # Principle point
        cx = 640/2
        cy = 480/2

        self.xo = self.dist*np.tan((self.xc - cx)/fx)*100 # Convert to cm
        self.yo = self.dist*np.tan((self.yc - cy)/fy)*100
        self.theta = np.arctan2(self.yo, self.xo)* (180 / np.pi)

        final.x = self.xo
        final.y = self.yo
        final.theta = self.theta

        self.pose_publisher_.publish(final)
        self.get_logger().info(f'Final Pose2D: x={self.xo}, y={self.yo}, theta={self.theta}')

def main(args=None):
    rclpy.init(args=args)

    get_object_range = GetObjectRange()
    rclpy.spin(get_object_range)
    get_object_range.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()