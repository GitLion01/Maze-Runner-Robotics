#!/usr/bin/env python3

import rospy
import math
import tf
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Point, Pose, PoseArray, Twist
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
import heapq

class Node:
    def __init__(self, position, parent=None):
        self.position = position
        self.parent = parent
        self.g_cost = 0.0
        self.h_cost = 0.0
        self.f_cost = 0.0

    def __eq__(self, other):
        return self.position == other.position

    def __lt__(self, other):
        return self.f_cost < other.f_cost

class RobotNavigator:
    def __init__(self):
        rospy.init_node('unknown_maze_navigator', anonymous=True)

        # Robot state
        self.current_position = None
        self.current_orientation = 0.0
        self.grid = None
        self.map_resolution = None
        self.map_origin = None
        self.goal_position = None
        self.path = []
        self.waypoints = []

        # Navigation parameters
        self.linear_speed = 0.2
        self.angular_speed = 0.4
        self.distance_threshold = 0.1
        self.angle_threshold = 0.2
        self.frontier_exploration = True

        # Subscribers
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.path_pub = rospy.Publisher('/planned_path', PoseArray, queue_size=10)

    def odom_callback(self, msg):
        # Update robot's position and orientation from odometry
        self.current_position = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        _, _, self.current_orientation = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

    def map_callback(self, msg):
        # Update map information dynamically
        self.map_resolution = msg.info.resolution
        self.map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        width = msg.info.width
        data = msg.data
        self.grid = [data[i:i + width] for i in range(0, len(data), width)]
        
        if self.frontier_exploration:
            self.explore()

    def scan_callback(self, msg):
        # Handle dynamic obstacle detection
        if self.detect_obstacle(msg):
            rospy.loginfo("Obstacle detected, replanning path...")
            self.plan_path()

    def detect_obstacle(self, msg):
        front_distance = min(msg.ranges[:10] + msg.ranges[-10:])  # Front region
        return front_distance < 0.5

    def detect_frontiers(self):
        # Detect unexplored frontiers on the map
        frontiers = []
        for y in range(1, len(self.grid) - 1):
            for x in range(1, len(self.grid[0]) - 1):
                if self.grid[y][x] == -1:  # Unknown cell
                    neighbors = [(x + dx, y + dy) for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]]
                    if any(self.grid[ny][nx] == 0 for nx, ny in neighbors):
                        frontiers.append((x, y))
        return frontiers

    def select_frontier(self, frontiers):
        # Select the closest frontier to explore
        robot_x = int((self.current_position.x - self.map_origin[0]) / self.map_resolution)
        robot_y = int((self.current_position.y - self.map_origin[1]) / self.map_resolution)
        return min(frontiers, key=lambda f: math.sqrt((f[0] - robot_x)**2 + (f[1] - robot_y)**2))

    def explore(self):
        # Perform frontier-based exploration
        frontiers = self.detect_frontiers()
        if not frontiers:
            rospy.logwarn("No frontiers found. Saving current position as a waypoint.")
            self.save_waypoint()
            return

        selected_frontier = self.select_frontier(frontiers)
        goal_x = self.map_origin[0] + selected_frontier[0] * self.map_resolution
        goal_y = self.map_origin[1] + selected_frontier[1] * self.map_resolution
        self.goal_position = (goal_x, goal_y)
        self.save_waypoint()
        self.plan_path()

    def save_waypoint(self):
        # Save the current position as a waypoint
        if self.current_position:
            x = self.current_position.x
            y = self.current_position.y
            self.waypoints.append((x, y))
            rospy.loginfo(f"Waypoint saved: ({x}, {y})")

    def go_to_previous_waypoint(self):
        # Navigate to the most recent waypoint
        if not self.waypoints:
            rospy.logwarn("No waypoints to return to.")
            return

        last_waypoint = self.waypoints.pop()
        self.goal_position = last_waypoint
        rospy.loginfo(f"Returning to waypoint: {last_waypoint}")
        self.plan_path()

    def plan_path(self):
        # Plan path to the goal using A*
        if not self.grid or not self.goal_position:
            rospy.logwarn("Map or goal not set, skipping path planning.")
            return

        start = self.get_robot_pixel_position()
        goal = self.get_goal_pixel_position()

        astar = AStar(start, goal, self.grid, self.manhattan_heuristic)
        self.path = astar.run()

        if self.path:
            rospy.loginfo("Path planned successfully.")
            self.publish_path()
        else:
            rospy.logwarn("Path planning failed.")

    def get_robot_pixel_position(self):
        x = int((self.current_position.x - self.map_origin[0]) / self.map_resolution)
        y = int((self.current_position.y - self.map_origin[1]) / self.map_resolution)
        return (x, y)

    def get_goal_pixel_position(self):
        x = int((self.goal_position[0] - self.map_origin[0]) / self.map_resolution)
        y = int((self.goal_position[1] - self.map_origin[1]) / self.map_resolution)
        return (x, y)

    def publish_path(self):
        # Publish the planned path as a PoseArray
        path_msg = PoseArray()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "map"
        for x, y in self.path:
            pose = Pose()
            pose.position.x = self.map_origin[0] + x * self.map_resolution
            pose.position.y = self.map_origin[1] + y * self.map_resolution
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)

    def manhattan_heuristic(self, current, goal):
        return abs(current[0] - goal[0]) + abs(current[1] - goal[1])

    def move_to_goal(self):
        # Move the robot along the planned path
        if not self.path:
            return

        current_goal = self.path.pop(0)
        goal_x = self.map_origin[0] + current_goal[0] * self.map_resolution
        goal_y = self.map_origin[1] + current_goal[1] * self.map_resolution

        while not rospy.is_shutdown():
            distance = math.sqrt((goal_x - self.current_position.x)**2 + (goal_y - self.current_position.y)**2)
            angle_to_goal = math.atan2(goal_y - self.current_position.y, goal_x - self.current_position.x)

            if distance < self.distance_threshold:
                rospy.loginfo("Reached goal point.")
                break

            cmd_vel = Twist()

            angle_diff = angle_to_goal - self.current_orientation
            if abs(angle_diff) > self.angle_threshold:
                cmd_vel.angular.z = self.angular_speed if angle_diff > 0 else -self.angular_speed
            else:
                cmd_vel.linear.x = self.linear_speed

            self.cmd_vel_pub.publish(cmd_vel)
            rospy.sleep(0.1)

    def run(self):
        # Main loop for navigation
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            if self.path:
                self.move_to_goal()
            rate.sleep()

class AStar:
    def __init__(self, start, goal, grid, heuristic):
        self.start = Node(start)
        self.goal = Node(goal)
        self.grid = grid
        self.heuristic = heuristic
        self.open_list = []
        self.closed_list = set()

    def run(self):
        self.start.g_cost = 0
        self.start.h_cost = self.heuristic(self.start.position, self.goal.position)
        self.start.f_cost = self.start.g_cost + self.start.h_cost
        heapq.heappush(self.open_list, self.start)

        while self.open_list:
            current = heapq.heappop(self.open_list)

            if current == self.goal:
                return self.reconstruct_path(current)

            self.closed_list.add(current)
            for neighbor in self.get_neighbors(current):
                if neighbor in self.closed_list:
                    continue

                tentative_g_cost = current.g_cost + 1  # Assume uniform cost
                if tentative_g_cost < neighbor.g_cost or neighbor not in self.open_list:
                    neighbor.g_cost = tentative_g_cost
                    neighbor.h_cost = self.heuristic(neighbor.position, self.goal.position)
                    neighbor.f_cost = neighbor.g_cost + neighbor.h_cost
                    neighbor.parent = current

                    if neighbor not in self.open_list:
                        heapq.heappush(self.open_list, neighbor)

        return []

    def get_neighbors(self, node):
        x, y = node.position
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        neighbors = []
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if 0 <= nx < len(self.grid[0]) and 0 <= ny < len(self.grid):
                if self.grid[ny][nx] == 0:  # Free cell
                    neighbors.append(Node((nx, ny)))
        return neighbors

    def reconstruct_path(self, node):
        path = []
        while node:
            path.append(node.position)
            node = node.parent
        path.reverse()
        return path

if __name__ == '__main__':
    try:
        navigator = RobotNavigator()
        navigator.run()
    except rospy.ROSInterruptException:
        pass
