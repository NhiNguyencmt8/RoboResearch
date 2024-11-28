import rospy
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Int8
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np
import time
from statistics import mode

class GoToGoal:

    def __init__(self):
        # Initialize the node
        rospy.init_node('go_to_goal', anonymous=True)

        # Subscribers and Publishers
        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sign_subscriber = rospy.Subscriber('/prediction', Int8, self.sign_callback)
        self.laserscan_subscriber = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        self.Init = True
        self.reached_goal = False
        self.odom = Odometry()
        self.average_distance = 1.5
        self.state = 0
        self.sign = ""
        self.sign_buffer = [4, 4, 4, 4, 4]

    def scan_callback(self, msg):
        self.average_distance = msg.ranges[0]
        self.average_distance_backward = msg.ranges[70]

    def sign_callback(self, msg):
        sign = msg.data
        if sign != 0:
            self.sign_buffer.append(sign)
        if len(self.sign_buffer) > 10:
            self.sign_buffer.pop(0)
        
        if sign == 0:
            self.sign = "wall"
        elif sign == 1:
            self.sign = "left"
        elif sign == 2:
            self.sign = "right"
        elif sign == 3:
            self.sign = "turnaround"
        elif sign == 4:
            self.sign = "stop"
        elif sign == 5:
            self.sign = "goal"

    def move(self):
        if self.state == 0:
            if self.sign == "left":
                self.goalAngle = self.globalAng + 1.52
                self.state = 2
            elif self.sign == "right":
                self.goalAngle = self.globalAng - 1.52
                self.state = 3
            elif self.sign == "wall":
                self.state = 4
            elif self.sign == "turnaround":
                self.goalAngle = self.globalAng + 3.12
                self.state = 5
            elif self.sign == "stop":
                self.goalAngle = self.globalAng + 3.12
                self.state = 6
            elif self.sign == "goal":
                self.state = 7
            else:
                self.state = 1

        elif self.state == 1:
            self.go_forward()
        elif self.state == 2:
            self.turn_left()
        elif self.state == 3:
            self.turn_right()
        elif self.state == 4:
            self.recovery_behavior()
        elif self.state == 5 or self.state == 6:
            self.turn_around()
        elif self.state == 7:
            self.reached_goal = True

    def turn_around(self):
        print("Turning 180")
        if self.globalAng < self.goalAngle:
            self.simple_velocity_controller(0.0, 0.3)
        else:
            self.simple_velocity_controller(0.0, 0.0)
            self.state = 1

    def go_forward(self):
        if self.average_distance > 0.5:
            self.simple_velocity_controller(0.12, 0.0)
        else:
            self.simple_velocity_controller(0.0, 0.0)
            self.state = 0

    def turn_left(self):
        if self.globalAng < self.goalAngle:
            self.simple_velocity_controller(0.0, 0.3)
        else:
            self.simple_velocity_controller(0.0, 0.0)
            self.state = 1

    def turn_right(self):
        if self.globalAng > self.goalAngle:
            self.simple_velocity_controller(0.0, -0.3)
        else:
            self.simple_velocity_controller(0.0, 0.0)
            self.state = 1

    def recovery_behavior(self):
        sign = mode(self.sign_buffer)
        if sign == 1:
            self.sign = "left"
            self.goalAngle = self.globalAng + 1.52
            self.state = 2
        elif sign == 2:
            self.sign = "right"
            self.goalAngle = self.globalAng - 1.52
            self.state = 3
        elif sign == 3:
            self.sign = "turnaround"
            self.goalAngle = self.globalAng + 3.12
            self.state = 5
        elif sign == 4:
            self.sign = "stop"
            self.goalAngle = self.globalAng + 3.12
            self.state = 5
        elif sign == 5:
            self.sign = "goal"
            self.state = 7

    def simple_velocity_controller(self, speed_x, speed_angular):
        velocity_msg = Twist()
        velocity_msg.angular.z = speed_angular
        velocity_msg.linear.x = speed_x
        self.velocity_publisher.publish(velocity_msg)

    def odom_callback(self, msg):
        self.update_odometry(msg)
        if self.reached_goal:
            rospy.loginfo("Goal Reached!!")
        else:
            self.move()

    def update_odometry(self, odom):
        position = odom.pose.pose.position
        q = odom.pose.pose.orientation
        orientation = np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

        if self.Init:
            self.Init = False
            self.Init_ang = orientation
            self.globalAng = self.Init_ang

        self.globalAng = orientation - self.Init_ang
        self.globalAng = np.arctan2(np.sin(self.globalAng), np.cos(self.globalAng))


def main():
    go_to_goal = GoToGoal()
    rospy.spin()

if __name__ == '__main__':
    main()
