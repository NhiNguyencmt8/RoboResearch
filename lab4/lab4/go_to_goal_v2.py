import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
import time
import math

class GoToGoal(Node):
    def __init__(self):
        super().__init__('go_to_goal_v2')
        
        custom_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )
        
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile=custom_qos_profile
        )
        self._subscriber = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.laser_scan_process, 
            qos_profile=custom_qos_profile)

        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.x_robot = 0.0
        self.y_robot = 0.0
        self.heading = 0.0

        # Initialize parameters 
        self.init_pos = np.zeros(3)  
        self.init_ang = 0.0          
        self.global_pos = np.zeros(3) 
        self.global_ang = 0.0      

        self.linear_speed = 0.4   
        self.angular_speed = 0.3
        self.min_distance = 0.15  # Minimum distance to consider the target reached
        self.Init = True  # Flag to indicate the first odometry reading

        # Target points for the robot to reach (x, y coordinates)
        self.target_points = [ (1.5,0), (1.5, -1.4), (0, -1.5)] #(1.5,0)
        self.current_target_index = 0  # Index of the current target point
        self.target_x = 0.0
        self.target_y = 0.0
        self.state = 'idle'
        self.obstacle_detected = False

    def laser_scan_process(self, msg):
        message_range = msg.ranges
        field_range = 60
        initial_angle = 330
        self.obstacle_detected = False

        # if len(message_range) > 345:
        #     range = (message_range[0] + message_range[15] + message_range[345])/3
        #     if range >0.6 and (self.target_x != 0 or self.target_y != 0):
        #         self.obstacle_detected = True
        #         self.get_logger().info('Now going into object avoidance')
        #         self.state = 'object avoidance'    

        # # Make sure initial_angle + field_range does not exceed the length of message_range
        for i in range(initial_angle, initial_angle + field_range):
            index = i % 360  # Wrap around using modulo
            if index < len(message_range) and message_range[index] < 0.5 and (self.target_x != 0 or self.target_y != 0):
                self.obstacle_detected = True
                self.get_logger().info('Now going into object avoidance')
                self.state = 'object avoidance'
                break
        
    def odom_callback(self, msg):
        position = msg.pose.pose.position
        q = msg.pose.pose.orientation

        # Convert quaternion to yaw (heading)
        orientation = np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

        if self.Init:
            # First odometry reading, set the initial position and orientation
            self.Init = False
            self.init_ang = orientation
            self.global_ang = self.init_ang

            # Rotation matrix to account for initial orientation
            Mrot = np.array([
                [np.cos(self.init_ang), np.sin(self.init_ang)],
                [-np.sin(self.init_ang), np.cos(self.init_ang)]
            ])

            # Transform the initial position to (0, 0, 0)
            self.init_pos[0] = Mrot[0, 0] * position.x + Mrot[0, 1] * position.y
            self.init_pos[1] = Mrot[1, 0] * position.x + Mrot[1, 1] * position.y
            self.init_pos[2] = position.z

            self.get_logger().info(f'Initialized pose: x={self.init_pos[0]:.2f}, y={self.init_pos[1]:.2f}, theta={self.init_ang:.2f}')

        # Apply the rotation matrix to transform the current position
        Mrot = np.array([
            [np.cos(self.init_ang), np.sin(self.init_ang)],
            [-np.sin(self.init_ang), np.cos(self.init_ang)]
        ])

        # Transform the current position to the global coordinate frame
        self.global_pos[0] = Mrot[0, 0] * position.x + Mrot[0, 1] * position.y - self.init_pos[0]
        self.global_pos[1] = Mrot[1, 0] * position.x + Mrot[1, 1] * position.y - self.init_pos[1]
        self.global_pos[2] = position.z - self.init_pos[2]

        # Adjust the yaw to the global frame
        self.global_ang = orientation - self.init_ang

        # Log the global pose (reset to (0, 0, 0) initially)
        self.get_logger().info(f'Reset Robot Pose: x={self.global_pos[0]:.2f}, y={self.global_pos[1]:.2f}, theta degrees={self.radians_to_degrees(self.global_ang):.2f}, theta rad={self.global_ang:.2f}')
        self.control_robot()

    def control_robot(self):
        # self.target_x, self.target_y = self.target_points[self.current_target_index]
        # distance_to_target = np.sqrt((self.target_x - self.global_pos[0]) ** 2 + (self.target_y - self.global_pos[1]) ** 2)
        # angle_to_target = self.radians_to_degrees(np.arctan2(self.target_y - self.global_pos[1], self.target_x - self.global_pos[0]))
        # angle_error = angle_to_target - self.global_ang
        # self.get_logger().info(f'Our angle: ( ), Angle to target={angle_to_target:.2f}, angle_error={angle_error:.2f}')
        
        # self.get_logger().info(f'Next pose: x={self.target_x:.2f}, y={self.target_y:.2f}')
        if self.state == 'idle':
            self.stop()

            if self.current_target_index < len(self.target_points):
                self.target_x, self.target_y = self.target_points[self.current_target_index]
                self.state = 'go to goal'

        elif self.state == 'go to goal':
            self.get_logger().info('Go to goal!!')
            self.get_logger().info(f'Next pose: x={self.target_x:.2f}, y={self.target_y:.2f}')

            if np.sqrt((self.target_x - self.global_pos[0]) ** 2 + (self.target_y - self.global_pos[1]) ** 2) <= self.min_distance + 0.01:
                if self.current_target_index == 2:
                    self.get_logger().info('Complete the iteration')
                    self.state = 'idle'
                else:
                    self.get_logger().info('At the point, moving to the next one')
                    self.current_target_index += 1
                    self.target_x, self.target_y = self.target_points[self.current_target_index]
                    
                    # angle_to_target = self.radians_to_degrees(np.arctan2(self.target_y - self.global_pos[1], self.target_x - self.global_pos[0]))
                    # angle_error = angle_to_target - (-self.radians_to_degrees(self.global_ang))
                    if self.target_y - 0.1 - self.global_pos[1] < 0 : #Fix this to angle later on
                        self.angular_speed = 0.175
                        self.get_logger().info('TUNRINGGG')
                        self.state = 'turning'

            else:
                self.angular_speed = 0.3
                self.get_logger().info('Go straight!!')
                self.state = 'go straight'

        elif self.state == 'go straight':
            self.go_straight()

        elif self.state == 'turning':
            self.turn_right_90_degrees()
        elif self.state == 'object avoidance':
            if self.obstacle_detected:
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = -0.3
                self.cmd_vel_publisher.publish(twist)
            else: 
                # twist = Twist()
                # twist.linear.x = 0.2
                # twist.angular.z = 0.3
                # self.cmd_vel_publisher.publish(twist)
                self.state = 'go to goal'




    def stop(self):
        self.get_logger().info('!!!!!!!!!!! STOP !!!!!!!!!!!!')
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)

    def turn_right_90_degrees(self):
        self.get_logger().info('>>>>>>> Turning 90 degrees to the right...')
        angle_to_target = self.radians_to_degrees(np.arctan2(self.target_y - self.global_pos[1], self.target_x - self.global_pos[0]))
        angle_error = angle_to_target - (self.radians_to_degrees(self.global_ang))
        self.get_logger().info(f'Robot Heading: {self.radians_to_degrees(self.global_ang):.2f}, Goal Heading: {angle_to_target:.2f}, Angle Error: {angle_error:.2f}')

        twist = Twist()
        twist.linear.x = 0.0
        if abs(angle_error) < 7:
            k = 0.05
        else:
            k = 0.02
        twist.angular.z = self.angular_speed * angle_error * k
        if abs(angle_error) < 5:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Finished turning 90 degrees to the right.')
            time.sleep(5)
            self.angular_speed = 0.05
            self.state = 'go to goal'
        
        self.get_logger().info(f'Angular velocity {twist.angular.z}')
        self.cmd_vel_publisher.publish(twist)

    def go_straight(self):
        distance_to_target = np.sqrt((self.target_x - self.global_pos[0]) ** 2 + (self.target_y - self.global_pos[1]) ** 2)
        angle_to_target = self.radians_to_degrees(np.arctan2(self.target_y - self.global_pos[1], self.target_x - self.global_pos[0]))
        angle_error = angle_to_target - (self.radians_to_degrees(self.global_ang))
        # Log angles and error
        
        twist = Twist()
        if abs(angle_error) > 0.0:
            k = 0.04
        else:
            k = 0.05
        twist.angular.z = k* self.angular_speed * angle_error
        self.get_logger().info(f'Distance to target: {distance_to_target:.2f}, Angle error {angle_error: .2f}, Angular velocity {twist.angular.z}')

        if distance_to_target < self.min_distance or (self.current_target_index == 2 and self.global_pos[0] < 0.0):
            twist.linear.x = 0.0
            self.cmd_vel_publisher.publish(twist)
            self.state = 'go to goal'
        else:
            if abs(distance_to_target) > 0.6:
                k = 0.2
            else:
                k = 0.5
            twist.linear.x = self.linear_speed*distance_to_target*k
        self.cmd_vel_publisher.publish(twist)

    def radians_to_degrees(self, radians):
        degrees = radians * (180 / math.pi)
        # Wrap the degrees to the range -180 to 180
        wrapped_degrees = (degrees + 180) % 360 - 180
        return wrapped_degrees

def main(args=None):
    rclpy.init(args=args)

    go_to_goal = GoToGoal()
    rclpy.spin(go_to_goal)

    go_to_goal.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
