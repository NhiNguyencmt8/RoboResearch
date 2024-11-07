import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from math import sqrt
import time

class WaypointPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_publisher')
        
        # Publisher to /goal_pose
        self.publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # Parameters
        self.waypoints = self.load_waypoints('/home/rutcss23/ros2_ws/src/lab5/lab5/waypoints.txt')
        self.current_waypoint_index = 0

        # Timer to check if the robot has reached the goal
        self.timer = self.create_timer(1.0, self.check_goal_reached)
        
        # Last published waypoint
        self.last_goal = None

    def load_waypoints(self, filename):
        waypoints = []
        with open(filename, 'r') as file:
            for line in file:
                x, y, z, w = map(float, line.strip().split(','))
                waypoints.append((x, y, z, w))
        return waypoints

    def publish_next_waypoint(self):
        # If all waypoints are published, stop the timer
        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info("All waypoints reached.")
            self.timer.cancel()
            return

        # Publish the next waypoint
        x, y, z, w = self.waypoints[self.current_waypoint_index]
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = w

        self.publisher_.publish(pose)
        self.get_logger().info(f'Published waypoint {self.current_waypoint_index + 1}: {pose}')

        # Update the last goal
        self.last_goal = pose
        self.current_waypoint_index += 1

    def check_goal_reached(self):
        # Here we assume the robot position could be retrieved from feedback or TF
        # For simplicity, we simulate the robot reaching the goal after a fixed delay
        if self.last_goal is None:
            self.publish_next_waypoint()
            return

        # Simulate reaching the goal after a delay (in a real scenario, check distance to goal)
        time.sleep(5)  # Simulated wait time to reach goal
        self.get_logger().info("Goal reached.")

        # Publish the next waypoint
        self.publish_next_waypoint()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
