import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
import time

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

        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.x_robot = 0.0
        self.y_robot = 0.0
        self.heading = 0.0

        # Initialize parameters 
        self.init_pos = np.zeros(3)  
        self.init_ang = 0.0          
        self.global_pos = np.zeros(3) 
        self.global_ang = 0.0      

        self.linear_speed = 0.1   
        self.angular_speed = 0.05
        self.min_distance = 0.05  # Minimum distance to consider the target reached
        self.Init = True  # Flag to indicate the first odometry reading

        # Target points for the robot to reach (x, y coordinates)
        self.target_points = [(1.5, 0), (1.5, 1.4), (0, 1.5)]
        self.current_target_index = 0  # Index of the current target point
        self.target_x = 0.0
        self.target_y = 0.0
        self.state = 'idle'

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
        self.get_logger().info(f'Reset Robot Pose: x={self.global_pos[0]:.2f}, y={self.global_pos[1]:.2f}, theta={self.global_ang:.2f}')
        self.control_robot()

def control_robot(self):
    self.get_logger().info('Controlling Milhouse')
    if self.state == 'idle':
        stop()
        if self.current_target_index < len(self.target_points):
            self.target_x, self.target_y = self.target_points[self.current_target_index]
            self.state == 'go to goal'

    elif self.state == 'go to goal':
        if np.sqrt((self.target_x - self.global_pos[0]) ** 2 + (self.target_y - self.global_pos[1]) ** 2) <= self.min_distance:
            if self.current_target_index < len(self.target_points):
                self.get_logger().info('Complete the iteration')
                self.state = 'idle'
            self.get_logger().info('At the point, moving to the next one')
            self.current_target_index += 1
            self.target_x, self.target_y = self.target_points[self.current_target_index]

        elif abs(self.target_x - self.global_pos[0]) <= 0.1:
            if self.target_y - self.global_pos[1] > 0.05:
                self.state = 'go straight'
            self.state = 'turning'
        else:
            self.state = 'go straight'

    elif self.state == 'go straight':
        if self.target_y - self.global_pos[1] > 0.05:
            go_straight()
        else: 
            self.state = 'go to goal'
    
    elif self.state == 'turning':
        turn_right_90_degrees()
        self.state = 'go to goal'




def stop(self):
    self.get_logger().info('!!!!!!!!!!! STOP !!!!!!!!!!!!')
    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    self.cmd_vel_publisher.publish(twist)


def turn_right_90_degrees(self):
    self.get_logger().info('>>>>>>>Turning 90 degrees to the right...')
    
    distance_to_target = np.sqrt((self.target_x - self.global_pos[0]) ** 2 + (self.target_y - self.global_pos[1]) ** 2)
    angle_to_target = np.arctan2(self.target_y - self.global_pos[1], self.target_x - self.global_pos[0])
    angle_error = angle_to_target - self.global_ang
    angle_error = (angle_error + np.pi) % (2 * np.pi) - np.pi
    self.get_logger().info(f'Turning to target: ({self.target_x}, {self.target_y}), distance={distance_to_target:.2f}, angle_error={angle_error:.2f}')

    # Set the angular velocity for turning to the right
    twist = Twist()
    twist.angular.z = -self.angular_speed  # Negative for right turn
    self.cmd_vel_publisher.publish(twist)

    # Duration to turn 90 degrees (adjust based on your robot's speed)
    turn_duration = np.pi / 2 / abs(twist.angular.z)  # 90 degrees in radians divided by speed
    time.sleep(turn_duration)
    

    # Stop the robot after the turn
    twist.angular.z = 0.0
    self.cmd_vel_publisher.publish(twist)
    self.get_logger().info('Finished turning 90 degrees to the right.')

def go_straight(self):
    distance_to_target = np.sqrt((self.target_x - self.global_pos[0]) ** 2 + (self.target_y - self.global_pos[1]) ** 2)
    angle_to_target = np.arctan2(self.target_y - self.global_pos[1], self.target_x - self.global_pos[0])
    angle_error = angle_to_target - self.global_ang

    # Normalize angle error to be between -pi and pi
    angle_error = (angle_error + np.pi) % (2 * np.pi) - np.pi
    self.get_logger().info(f'<<<<<<<Going straight to target: ({self.target_x}, {self.target_y}), distance={distance_to_target:.2f}, angle_error={angle_error:.2f}')
    twist = Twist()
    twist.angular.z = self.angular_speed * angle_error
    twist.linear.x = self.linear_speed if distance_to_target > self.min_distance else 0.0




def main(args=None):
    rclpy.init(args=args)

    go_to_goal = GoToGoal()
    rclpy.spin(go_to_goal)

    go_to_goal.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()