#!/usr/bin/env python3

import sys
import os
import numpy as np
import yaml
import cv2

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose, Twist


class Navigation(Node):
    """! Navigation node class."""

    def __init__(self, node_name='Navigation'):
        super().__init__(node_name)
        self.get_logger().info('Initializing Navigation Node...')
        self.path = Path()
        self.goal_pose = PoseStamped()
        self.ttbot_pose = PoseStamped()

        # Load the map using os.path.join for correct path handling
        map_yaml_path = '/home/me597/Desktop/sync_classroom_map.yaml'
        self.get_logger().info(f'Loading map from {map_yaml_path}...')
        self.map_grid, self.map_resolution, self.map_origin = self.load_map(map_yaml_path)
        self.get_logger().info('Map loaded successfully.')

        # Subscribers
        self.get_logger().info('Setting up subscriptions...')
        self.create_subscription(PoseStamped, '/move_base_simple/goal', self.__goal_pose_cbk, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.__ttbot_pose_cbk, 10)

        # Publishers
        self.get_logger().info('Setting up publishers...')
        self.path_pub = self.create_publisher(Path, 'global_plan', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.rate = self.create_rate(10)

    def __goal_pose_cbk(self, data):
        self.goal_pose = data
        self.get_logger().info(
            'Received new goal pose: {:.4f}, {:.4f}'.format(self.goal_pose.pose.position.x, self.goal_pose.pose.position.y))

    def __ttbot_pose_cbk(self, data):
        self.ttbot_pose = data.pose
        self.get_logger().info(
            'Updated TurtleBot pose: {:.4f}, {:.4f}'.format(self.ttbot_pose.pose.position.x, self.ttbot_pose.pose.position.y))

    def load_map(self, map_yaml_file):
        """! Load map from .yaml and .pgm files."""
        with open(map_yaml_file, 'r') as file:
            map_metadata = yaml.safe_load(file)

        # Build the correct path for the image file
        map_image_path = os.path.join(os.path.dirname(map_yaml_file), map_metadata['image'])
        
        # Load the PGM file
        map_image = cv2.imread(map_image_path, cv2.IMREAD_GRAYSCALE)
        if map_image is None:
            self.get_logger().error(f"Failed to load map image from {map_image_path}")
            raise FileNotFoundError(f"Map image file not found at {map_image_path}")
        
        # Use occupied_thresh and free_thresh to set obstacle and free space
        occupied_thresh = map_metadata['occupied_thresh'] * 255  # scale to match 0-255 range of image
        free_thresh = map_metadata['free_thresh'] * 255  # scale to match 0-255 range of image

        grid = np.zeros_like(map_image, dtype=int)

        # Mark cells as obstacles or free space
        grid[map_image > occupied_thresh] = 1  # Occupied space (obstacles)
        grid[map_image < free_thresh] = 0  # Free space

        resolution = map_metadata['resolution']
        origin = map_metadata['origin']
        return grid, resolution, origin

    def a_star_path_planner(self, start_pose, end_pose):
        self.get_logger().info('Starting A* path planning...')
        path = Path()
        self.get_logger().info(
            'A* planner initialized.\n> start: {},\n> end: {}'.format(start_pose.pose.position, end_pose.pose.position))

        start = (int((start_pose.pose.position.x - self.map_origin[0]) / self.map_resolution),
                 int((start_pose.pose.position.y - self.map_origin[1]) / self.map_resolution))
        goal = (int((end_pose.pose.position.x - self.map_origin[0]) / self.map_resolution),
                int((end_pose.pose.position.y - self.map_origin[1]) / self.map_resolution))

        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

        open_set = {start: 0}
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        while open_set:
            current = min(open_set, key=lambda node: f_score[node])

            if current == goal:
                self.get_logger().info('Path found by A* algorithm.')
                path.poses = self.reconstruct_path(came_from, current)
                return path

            open_set.pop(current)
            for direction in directions:
                neighbor = (current[0] + direction[0], current[1] + direction[1])

                if not (0 <= neighbor[0] < self.map_grid.shape[0] and
                        0 <= neighbor[1] < self.map_grid.shape[1]) or self.map_grid[neighbor] == 1:
                    continue

                tentative_g_score = g_score[current] + 1

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + self.heuristic(neighbor, goal)
                    if neighbor not in open_set:
                        open_set[neighbor] = f_score[neighbor]

        self.get_logger().error('A* failed to find a path.')
        return path

    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def reconstruct_path(self, came_from, current):
        self.get_logger().info('Reconstructing path from A* algorithm...')
        path = []
        while current in came_from:
            pose = PoseStamped()
            pose.pose.position.x = current[0]
            pose.pose.position.y = current[1]
            path.append(pose)
            current = came_from[current]
        path.reverse()
        self.get_logger().info('Path reconstruction complete.')
        return path

    def get_path_idx(self, path, vehicle_pose):
        min_dist = float('inf')
        idx = 0
        for i, pose in enumerate(path.poses):
            dist = np.sqrt((pose.pose.position.x - vehicle_pose.pose.position.x) ** 2 +
                           (pose.pose.position.y - vehicle_pose.pose.position.y) ** 2)
            if dist < min_dist:
                min_dist = dist
                idx = i
        self.get_logger().info('Next waypoint index in path: {}'.format(idx))
        return idx

    def path_follower(self, vehicle_pose, current_goal_pose):
        self.get_logger().info('Following path...')
        dx = current_goal_pose.pose.position.x - vehicle_pose.pose.position.x
        dy = current_goal_pose.pose.position.y - vehicle_pose.pose.position.y

        desired_heading = np.arctan2(dy, dx)
        speed = 0.5

        orientation_q = vehicle_pose.pose.orientation
        current_heading = np.arctan2(
            2.0 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y),
            1.0 - 2.0 * (orientation_q.y ** 2 + orientation_q.z ** 2))

        heading_error = desired_heading - current_heading
        heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))

        angular_velocity = 2.0 * heading_error

        self.get_logger().info('Speed: {:.2f}, Angular Velocity: {:.2f}'.format(speed, angular_velocity))
        return speed, angular_velocity

    def move_ttbot(self, speed, heading):
        self.get_logger().info('Sending movement commands...')
        cmd_vel = Twist()
        cmd_vel.linear.x = speed
        cmd_vel.angular.z = heading
        self.cmd_vel_pub.publish(cmd_vel)

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            self.get_logger().info('Creating path...')
            path = self.a_star_path_planner(self.ttbot_pose, self.goal_pose)

            # Check if the path is empty
            if not path.poses:
                self.get_logger().warn('Generated path is empty, skipping navigation step.')
                continue  # Skip this loop iteration if path is empty

            # Publish the path to RViz2 for visualization
            self.path_pub.publish(path)
            self.get_logger().info('Path created. Published to /global_plan.')

            # Ensure the index is within bounds
            idx = self.get_path_idx(path, self.ttbot_pose)
            if idx >= len(path.poses):
                self.get_logger().warn(f'Invalid index {idx}. Adjusting to the last pose in the path.')
                idx = len(path.poses) - 1  # Use the last pose if the index is out of bounds

            current_goal = path.poses[idx]
            speed, heading = self.path_follower(self.ttbot_pose, current_goal)
            self.move_ttbot(speed, heading)

            self.rate.sleep()


def main(args=None):
    rclpy.init(args=args)
    nav = Navigation(node_name='Navigation')

    try:
        nav.run()
    except KeyboardInterrupt:
        pass
    finally:
        nav.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
