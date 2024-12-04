#!/usr/bin/env python3

import sys
import os
import numpy as np
import heapq

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose, Twist


class Task2(Node):
    """! Task 2 Node.
    This node implements navigation with static obstacles using the A* algorithm.
    """

    def __init__(self, node_name='Task2'):
        super().__init__(node_name)
        self.path = Path()
        self.goal_pose = PoseStamped()
        self.ttbot_pose = PoseStamped()

        # Subscribers
        self.create_subscription(PoseStamped, '/move_base_simple/goal', self.__goal_pose_cbk, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.__ttbot_pose_cbk, 10)

        # Publishers
        self.path_pub = self.create_publisher(Path, 'global_plan', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Node rate
        self.rate = self.create_rate(10)

    def __goal_pose_cbk(self, data):
        self.goal_pose = data
        self.get_logger().info(
            'Goal pose received: {:.4f}, {:.4f}'.format(self.goal_pose.pose.position.x, self.goal_pose.pose.position.y))

    def __ttbot_pose_cbk(self, data):
        self.ttbot_pose = data.pose
        self.get_logger().info(
            'Turtlebot pose updated: {:.4f}, {:.4f}'.format(self.ttbot_pose.pose.position.x, self.ttbot_pose.pose.position.y))

    def a_star_path_planner(self, start_pose, end_pose):
        """! A* Path Planner.
        @param start_pose  PoseStamped containing the start position.
        @param end_pose    PoseStamped containing the goal position.
        @return Path object containing the sequence of waypoints.
        """
        # Example A* implementation
        start = (int(start_pose.pose.position.x), int(start_pose.pose.position.y))
        goal = (int(end_pose.pose.position.x), int(end_pose.pose.position.y))

        # Grid and cost placeholder
        grid = np.zeros((20, 20))  # Adjust grid size as necessary
        grid[5:10, 5:10] = 1  # Example obstacle area
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        cost_so_far = {start: 0}

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                break

            neighbors = self.get_neighbors(current, grid.shape)
            for next_node in neighbors:
                new_cost = cost_so_far[current] + 1
                if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                    cost_so_far[next_node] = new_cost
                    priority = new_cost + self.heuristic(goal, next_node)
                    heapq.heappush(open_set, (priority, next_node))
                    came_from[next_node] = current

        # Reconstruct path
        path = Path()
        current = goal
        while current in came_from:
            pose = PoseStamped()
            pose.pose.position.x = current[0]
            pose.pose.position.y = current[1]
            path.poses.append(pose)
            current = came_from[current]

        path.poses.reverse()
        return path

    def get_neighbors(self, current, grid_shape):
        """Returns neighbors of a grid cell."""
        x, y = current
        neighbors = []
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nx, ny = x + dx, y + dy
            if 0 <= nx < grid_shape[0] and 0 <= ny < grid_shape[1]:
                neighbors.append((nx, ny))
        return neighbors

    def heuristic(self, a, b):
        """Calculates Manhattan distance heuristic."""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def path_follower(self, vehicle_pose, current_goal_pose):
        """Path Follower.
        @param vehicle_pose        Current pose of the turtlebot.
        @param current_goal_pose   Next waypoint in the path.
        @return Speed and heading commands.
        """
        dx = current_goal_pose.pose.position.x - vehicle_pose.position.x
        dy = current_goal_pose.pose.position.y - vehicle_pose.position.y
        distance = np.sqrt(dx ** 2 + dy ** 2)
        heading = np.arctan2(dy, dx)

        speed = min(0.5, distance)  # Cap the speed
        return speed, heading

    def move_ttbot(self, speed, heading):
        cmd_vel = Twist()
        cmd_vel.linear.x = speed
        cmd_vel.angular.z = heading
        self.cmd_vel_pub.publish(cmd_vel)

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            path = self.a_star_path_planner(self.ttbot_pose, self.goal_pose)
            idx = 0

            while idx < len(path.poses):
                current_goal = path.poses[idx]
                speed, heading = self.path_follower(self.ttbot_pose.pose, current_goal)
                self.move_ttbot(speed, heading)

                idx += 1
                self.rate.sleep()


def main(args=None):
    rclpy.init(args=args)
    task2 = Task2(node_name='Task2')

    try:
        task2.run()
    except KeyboardInterrupt:
        pass
    finally:
        task2.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
