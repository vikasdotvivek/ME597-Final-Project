import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
import math
import heapq  # For A* priority queue


class Task2Navigator(Node):
    def __init__(self):
        super().__init__('task2_navigator')

        # Publishers and Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)

        # TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Variables
        self.map_data = None
        self.goal_pose = None
        self.robot_pose = None
        self.path = []

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)

    def map_callback(self, msg):
        """Receive and store map data."""
        self.map_data = msg

    def goal_callback(self, msg):
        """Receive and store the goal pose."""
        self.goal_pose = msg
        self.get_logger().info(f"New goal received: {msg.pose.position}")

    def get_robot_pose(self):
        """Get the robot's current pose in the map frame."""
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            pose = PoseStamped()
            pose.header.frame_id = 'base_link'
            pose.pose.orientation.w = 1.0  # Identity orientation
            return do_transform_pose(pose, transform).pose
        except Exception as e:
            self.get_logger().error(f"Failed to get robot pose: {e}")
            return None

    def a_star_planner(self, start, goal):
        """Implement A* path planning."""
        def heuristic(p1, p2):
            return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

        def get_neighbors(node):
            directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
            neighbors = []
            for d in directions:
                nx, ny = node[0] + d[0], node[1] + d[1]
                if 0 <= nx < width and 0 <= ny < height and map_data[ny * width + nx] == 0:
                    neighbors.append((nx, ny))
            return neighbors

        map_data = self.map_data.data
        resolution = self.map_data.info.resolution
        origin_x = self.map_data.info.origin.position.x
        origin_y = self.map_data.info.origin.position.y
        width = self.map_data.info.width
        height = self.map_data.info.height

        # Convert start and goal to grid coordinates
        start = ((start[0] - origin_x) / resolution, (start[1] - origin_y) / resolution)
        goal = ((goal[0] - origin_x) / resolution, (goal[1] - origin_y) / resolution)

        # A* search
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: heuristic(start, goal)}

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                # Reconstruct path
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.reverse()
                return path

            for neighbor in get_neighbors(current):
                tentative_g_score = g_score[current] + heuristic(current, neighbor)
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return []

    def follow_path(self):
        """Follow the planned path using a controller."""
        if not self.path:
            return

        next_goal = self.path.pop(0)
        twist = Twist()
        twist.linear.x = 0.2  # Adjust speed
        twist.angular.z = 0.5  # Adjust turn rate
        self.cmd_vel_pub.publish(twist)

    def control_loop(self):
        """Main control loop for navigation."""
 #       if self.map_data is None or self.goal_pose is None:
 #           return

        self.robot_pose = self.get_robot_pose()
        if self.robot_pose is None:
            return

        # Plan path if no path exists
        if not self.path:
            start = (self.robot_pose.position.x, self.robot_pose.position.y)
            goal = (self.goal_pose.pose.position.x, self.goal_pose.pose.position.y)
            self.path = self.a_star_planner(start, goal)

            #hardcoded dummy path:
            self.path = [(1.0, 1.0), (2.0, 2.0), (3.0, 3.0)]

            if self.path:
                self.get_logger().info(f"Path planned with {len(self.path)} waypoints.")
            else:
                self.get_logger().warn("Failed to plan path.")

        # Follow the path


        self.follow_path()
        #self.get_logger().info("should follow path")


def main(args=None):
    rclpy.init(args=args)
    node = Task2Navigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
