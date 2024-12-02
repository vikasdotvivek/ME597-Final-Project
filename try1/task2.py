import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from tf_transformations import euler_from_quaternion
import numpy as np
import math
from queue import PriorityQueue


class AStarNavigator(Node):
    def __init__(self):
        super().__init__('a_star_navigator')

        # Publishers and Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/amcl_pose', self.pose_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)

        # Variables
        self.map_data = None
        self.robot_pose = None
        self.goal_pose = None
        self.path = []
        self.resolution = None
        self.origin = None

    def map_callback(self, msg):
        """Callback to receive the occupancy grid map."""
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.resolution = msg.info.resolution
        self.origin = (msg.info.origin.position.x, msg.info.origin.position.y)

    def pose_callback(self, msg):
        """Callback to receive the robot's pose from AMCL."""
        position = msg.pose.position
        orientation = msg.pose.orientation
        yaw = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )[2]
        self.robot_pose = (position.x, position.y, yaw)

    def goal_callback(self, msg):
        """Callback to receive a new goal pose."""
        self.goal_pose = (msg.pose.position.x, msg.pose.position.y)
        if self.map_data is not None and self.robot_pose is not None:
            self.plan_path()

    def plan_path(self):
        """Plan a path from the current position to the goal using A*."""
        start = self.world_to_grid(self.robot_pose[:2])
        goal = self.world_to_grid(self.goal_pose)

        # Run A* to generate the path
        path = self.a_star(start, goal)
        if path:
            self.path = [self.grid_to_world(p) for p in path]
            self.follow_path()
        else:
            self.get_logger().info("No path found to the goal.")

    def a_star(self, start, goal):
        """A* pathfinding algorithm."""
        open_set = PriorityQueue()
        open_set.put((0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        while not open_set.empty():
            _, current = open_set.get()

            if current == goal:
                return self.reconstruct_path(came_from, current)

            neighbors = self.get_neighbors(current)
            for neighbor in neighbors:
                tentative_g_score = g_score[current] + 1  # Assume cost of 1 for neighbors
                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    if neighbor not in [item[1] for item in open_set.queue]:
                        open_set.put((f_score[neighbor], neighbor))

        return None

    def heuristic(self, a, b):
        """Heuristic for A*: Euclidean distance."""
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    def get_neighbors(self, node):
        """Get neighbors of a grid cell."""
        neighbors = [
            (node[0] + dx, node[1] + dy)
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]
        ]
        return [
            n for n in neighbors
            if 0 <= n[0] < self.map_data.shape[0]
            and 0 <= n[1] < self.map_data.shape[1]
            and self.map_data[n[1], n[0]] == 0
        ]

    def reconstruct_path(self, came_from, current):
        """Reconstruct the path from A* search."""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def world_to_grid(self, position):
        """Convert world coordinates to grid indices."""
        x, y = position
        gx = int((x - self.origin[0]) / self.resolution)
        gy = int((y - self.origin[1]) / self.resolution)
        return gx, gy

    def grid_to_world(self, index):
        """Convert grid indices to world coordinates."""
        gx, gy = index
        x = self.origin[0] + gx * self.resolution
        y = self.origin[1] + gy * self.resolution
        return (x, y)

    def follow_path(self):
        """Follow the planned path using proportional control."""
        for waypoint in self.path:
            while not self.reach_waypoint(waypoint):
                twist = Twist()
                dx = waypoint[0] - self.robot_pose[0]
                dy = waypoint[1] - self.robot_pose[1]
                angle_to_goal = math.atan2(dy, dx)
                angle_error = angle_to_goal - self.robot_pose[2]

                twist.linear.x = 0.2
                twist.angular.z = 0.5 * angle_error
                self.cmd_vel_pub.publish(twist)

    def reach_waypoint(self, waypoint):
        """Check if the robot has reached a waypoint."""
        distance = math.sqrt(
            (self.robot_pose[0] - waypoint[0])**2 + (self.robot_pose[1] - waypoint[1])**2
        )
        return distance < 0.1


def main(args=None):
    rclpy.init(args=args)
    node = AStarNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

