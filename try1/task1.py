import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import math
import random

class AutonomousMapper(Node):
    def __init__(self):
        super().__init__('autonomous_mapper')

        # Publishers and Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)

        # Initialize variables
        self.laser_data = None
        self.map_data = None
        self.covered_cells = set()
        self.state = 'wall_following'

        # Timer for periodic updates
        self.timer = self.create_timer(0.1, self.control_loop)

        #self.toggle_direction = True

    def laser_callback(self, msg):
        self.laser_data = msg

    def map_callback(self, msg):
        self.map_data = msg
        self.update_coverage(msg)

    def update_coverage(self, map_msg):
        """Track covered grid cells using the map's occupancy grid."""
        resolution = map_msg.info.resolution
        origin_x = map_msg.info.origin.position.x
        origin_y = map_msg.info.origin.position.y
        width = map_msg.info.width
        height = map_msg.info.height

        for y in range(height):
            for x in range(width):
                idx = x + y * width
                if map_msg.data[idx] == 0:  # Free space
                    grid_x = origin_x + x * resolution
                    grid_y = origin_y + y * resolution
                    self.covered_cells.add((grid_x, grid_y))

    def control_loop(self):
        """Main control loop for autonomous navigation."""
        if self.laser_data is None:
            return

        twist = Twist()

        #self.get_logger().info(f"{self.laser_data.ranges}")

        # Wall following logic
        if self.state == 'wall_following':
            SAFE_DISTANCE = 0.8  # Configurable threshold

            if not self.laser_data.ranges or len(self.laser_data.ranges) == 0:
                twist.angular.z = 0.5
                twist.linear.x = 0.0
                self.get_logger().info("No laser data available")
            elif all(distance == float('inf') for distance in self.laser_data.ranges):
                twist.angular.z = -1.0  # Rotate in place if no obstacles are detected
                self.get_logger().info("All ranges are infinity (open space)")
            elif min(self.laser_data.ranges[0],self.laser_data.ranges[1],self.laser_data.ranges[5]) < SAFE_DISTANCE:
                # Too close to the wall, turn away
                error = SAFE_DISTANCE - min(self.laser_data.ranges)
                #twist.angular.z = -1.0 * error  # Smooth turn
                twist.angular.z = random.choice([0.75, -0.75])
                #if self.toggle_direction:
                 #   twist.angular.z = 0.75
                #else:
                 #   twist.angular.z = -0.75

                #self.toggle_direction = not self.toggle_direction   #flipping the damn toggle to get it fcking working wtf #also do you read comments?

                twist.linear.x = -0.2
                self.get_logger().info(f"Too close to the wall: {min(self.laser_data.ranges):.2f} meters")
            else:
                # Normal wall-following movement
                twist.linear.x = 0.5
                twist.angular.z = 0.0
                self.get_logger().info(f"Wall following: Closest distance = {min(self.laser_data.ranges):.2f} meters")

            # Occasionally switch to random bouncing
            if random.random() < 0.1:
                self.state = 'random_bouncing'
                if min(self.laser_data.ranges) < SAFE_DISTANCE:
                    self.state = 'wall_following'

        elif self.state == 'random_bouncing':
            twist.linear.x = random.uniform(0.05, 0.2)
            twist.angular.z = random.uniform(-1.0, 1.0)
            self.get_logger().info("Random bouncing")

            # Occasionally switch back to wall following
            if random.random() < 0.1:
                self.state = 'wall_following'

        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

