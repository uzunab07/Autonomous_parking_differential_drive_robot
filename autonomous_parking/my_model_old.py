from rclpy.node import Node
import rclpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
import numpy as np
from tf_transformations import euler_from_quaternion
from math import cos, sin, pi, atan2, sqrt
import sys
import os
sys.path.append(os.getcwd() + '/src/autonomous_parking/autonomous_parking')
from path_finder import get_path

def angle_from_quaternion(orientation):
    """Convert quaternion to Euler angles and return yaw."""
    qx = orientation.pose.pose.orientation.x
    qy = orientation.pose.pose.orientation.y
    qz = orientation.pose.pose.orientation.z
    qw = orientation.pose.pose.orientation.w

    # Convert quaternion to Euler angles
    _, _, yaw = euler_from_quaternion([qx, qy, qz, qw])
    return yaw

def generate_dense_path(waypoints, num_points=10):
    """
    Generate a dense path with linear interpolation between waypoints.

    :param waypoints: List of (x, y) tuples representing the key waypoints.
    :param num_points: Number of points to generate between each pair of waypoints.
    :return: List of dense waypoints.
    """
    dense_path = []
    for i in range(len(waypoints) - 1):
        x1, y1 = waypoints[i]
        x2, y2 = waypoints[i + 1]

        # Linearly interpolate between waypoints
        for t in np.linspace(0, 1, num_points):
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            dense_path.append((x, y))

    dense_path.append(waypoints[-1])  # Include the final waypoint
    return dense_path

def generate_obstacles(num_points, x_range, y_range):
    """
    Generate obstacles

    :param num_points: Number of waypoints to generate.
    :param x_range: Tuple specifying the range of x-coordinates (xmin, xmax).
    :param y_range: Tuple specifying the range of y-coordinates (ymin, ymax).
    :return: List of waypoints 
    """
    waypoints = []
    for _ in range(num_points):
        x = np.random.uniform(x_range[0], x_range[1])
        y = np.random.uniform(y_range[0], y_range[1])
        waypoints.append((x, y))
    return waypoints


class State:
    """Class to hold the robot's state."""
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta


class Control:
    """Class to hold control signals."""
    def __init__(self, v, w):
        self.v = v  # Linear velocity
        self.w = w  # Angular velocity



class MyBot(Node):
    def __init__(self,waypoint):
        super().__init__("My_Robot")

        # Robot state
        self.state = None

        # Path parameters
        self.waypoints = waypoint # Example path
        self.current_waypoint_idx = 0
        self.goal_tolerance = 0.1  # Distance to consider the waypoint reached

        # Velocity limits
        self.max_linear_velocity = 0.2
        self.max_angular_velocity = 1.0

        # Initialize ROS interfaces
        self.subscriber_ = self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile_sensor_data)
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.control_timer = self.create_timer(0.1, self.follow_path)
        self.print_timer = self.create_timer(3, self.print_state)

    def follow_path(self):
        """Follow the path using Pure Pursuit logic."""
        if self.state is None or self.current_waypoint_idx >= len(self.waypoints):
            return

        # Get the current waypoint
        goal_x, goal_y = self.waypoints[self.current_waypoint_idx]

        # Calculate distance and angle to the waypoint
        dx = goal_x - self.state.x
        dy = goal_y - self.state.y
        distance_to_goal = sqrt(dx**2 + dy**2)
        angle_to_goal = atan2(dy, dx)

        # Check if the waypoint is reached
        if distance_to_goal < self.goal_tolerance:
            self.get_logger().info(f"Reached waypoint {self.current_waypoint_idx + 1}")
            self.current_waypoint_idx += 1
            if self.current_waypoint_idx >= len(self.waypoints):
                self.get_logger().info("All waypoints reached. Stopping.")
                self.stop_robot()
            return

        # Compute control signals
        linear_velocity = min(self.max_linear_velocity, distance_to_goal) #if the distance to the goal< max vel slow down
        angular_velocity = 1.5 * self.normalize_angle(angle_to_goal - self.state.theta)

        # Clamp angular velocity
        angular_velocity = max(-self.max_angular_velocity, min(self.max_angular_velocity, angular_velocity))

        # Send velocity commands
        self.send_velocity(linear_velocity, angular_velocity)

    def send_velocity(self, linear_vel, angular_vel):
        """Publish velocity commands to move the robot."""
        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        self.vel_publisher.publish(twist)

    def stop_robot(self):
        """Stop the robot."""
        self.send_velocity(0.0, 0.0)

    @staticmethod
    def normalize_angle(angle):
        """Normalize angle to the range [-pi, pi]."""
        return (angle + pi) % (2 * pi) - pi

    def odom_callback(self, data: Odometry):
        """Callback to update the robot's state based on odometry data."""
        # Extract position and orientation
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        theta = angle_from_quaternion(data)

        # Initialize or update the robot's state
        if self.state is None:
            self.state = State(x, y, theta)
            self.get_logger().info("Initialized robot state.")
        else:
            self.state.x = x
            self.state.y = y
            self.state.theta = theta

    def print_state(self):
        """Print the current state of the robot."""
        if self.state is not None:
            self.get_logger().info(f"State: x={self.state.x:.2f}, y={self.state.y:.2f}, theta={self.state.theta:.2f}")


def main():
    rclpy.init()
    start = (0,0)
    goal = (5,3)
    path = get_path(start, goal)
    print(os.getcwd())
    # waypoints = [(0.0, 0.0), (2.0, 2.0), (3.0, 2.0), (4.3, 4.2)]
    # path = generate_dense_path(waypoints, 10)
    # print(f"Path: {path}")
    my_bot = MyBot(path)
    rclpy.spin(my_bot)
    my_bot.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
