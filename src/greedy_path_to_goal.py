#!/usr/bin/env python3

import rospy
import math
import numpy as np
import heapq
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from tf.transformations import euler_from_quaternion

# Konstanten für Umrechnung und Geschwindigkeit
DEG2RAD = math.pi / 180.0
LINEAR_VELOCITY = 0.2  # Vorwärtsgeschwindigkeit
ANGULAR_VELOCITY = 0.5  # Drehgeschwindigkeit


class Pose2D():
    x = 0
    y = 0
    theta = 0


class Turtlebot3Drive:
    def __init__(self):
        rospy.init_node('turtlebot3_drive', anonymous=True)

        # Initialisierung der Variablen
        self.pose = Pose2D()
        self.scan_data = [float('inf'), float('inf'), float('inf')]  # [CENTER, LEFT, RIGHT]
        self.goal = Point(-4.55, 1.0, 0.0)
        self.rate = rospy.Rate(10)

        # Sicherheitsabstände
        self.safe_distance = 0.3  # Sicherheitsabstand zu Hindernissen

        # Initialisiere Variablen für die Karte
        self.map_data = None
        self.costmap_2d = None
        self.map_resolution = None
        self.map_origin_x = None
        self.map_origin_y = None

        # Publisher und Subscriber initialisieren
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber("scan", LaserScan, self.laser_scan_callback)
        rospy.Subscriber("odom", Odometry, self.odom_callback)
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        quaternion = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, self.pose.theta = euler_from_quaternion(quaternion)
        self.pose.x = position.x
        self.pose.y = position.y

    def laser_scan_callback(self, msg):
        scan_angles = [0, 30, 330]
        for i, angle in enumerate(scan_angles):
            if math.isinf(msg.ranges[angle]):
                self.scan_data[i] = msg.range_max
            else:
                self.scan_data[i] = msg.ranges[angle]

    def map_callback(self, msg):
        self.map_data = msg
        self.map_resolution = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y

        try:
            width = msg.info.width
            height = msg.info.height
            self.costmap_2d = np.array(msg.data).reshape((height, width))
            rospy.loginfo(f"Map received with resolution: {self.map_resolution}, origin: ({self.map_origin_x}, {self.map_origin_y})")
        except Exception as e:
            rospy.logerr(f"Error creating costmap: {e}")

    def update_command_velocity(self, linear, angular):
        rospy.loginfo(f"Setting velocity - Linear: {linear}, Angular: {angular}")
        cmd_vel = Twist()
        cmd_vel.linear.x = linear
        cmd_vel.angular.z = angular
        self.cmd_vel_pub.publish(cmd_vel)

    def rotate_to(self, angle):
        """Rotate the robot to a specific angle."""
        angle = angle % (2 * math.pi)
        direction = 1
        angle_to_turn = abs(angle - self.pose.theta)

        # Minimum turning angle
        if angle_to_turn > math.pi:
            angle_to_turn = 2 * math.pi - angle_to_turn

        # Determine turning direction
        if angle != 3 * math.pi / 2:
            if angle < self.pose.theta <= angle + math.pi:
                direction = -1

        rospy.loginfo(f"Rotating to angle: {math.degrees(angle)}°")
        while angle_to_turn > 0.05 and not rospy.is_shutdown():
            msg = Twist()
            msg.angular.z = direction * ANGULAR_VELOCITY
            self.cmd_vel_pub.publish(msg)
            angle_to_turn = abs(angle - self.pose.theta)
            self.rate.sleep()

        self.update_command_velocity(0.0, 0.0)

    def forward(self, distance):
        """Move the robot forward while avoiding obstacles."""
        x_init = self.pose.x
        y_init = self.pose.y
        travelled_distance = 0

        rospy.loginfo(f"Moving forward {distance} meters.")
        while distance - travelled_distance > 0.05 and not rospy.is_shutdown():
            msg = Twist()
            if self.scan_data[0] < self.safe_distance:  # Obstacle ahead
                rospy.logwarn("Obstacle ahead! Avoiding...")
                if self.scan_data[1] < self.safe_distance:
                    msg.angular.z = -ANGULAR_VELOCITY
                elif self.scan_data[2] < self.safe_distance:
                    msg.angular.z = ANGULAR_VELOCITY
                else:
                    if self.scan_data[1] > self.scan_data[2]:
                        msg.angular.z = ANGULAR_VELOCITY
                    else:
                        msg.angular.z = -ANGULAR_VELOCITY
            else:
                msg.linear.x = LINEAR_VELOCITY
                msg.angular.z = 0

            self.cmd_vel_pub.publish(msg)
            travelled_distance = math.sqrt((self.pose.x - x_init)**2 + (self.pose.y - y_init)**2)
            self.rate.sleep()

        self.update_command_velocity(0.0, 0.0)

    def move(self, current, waypoint):
        """Move to a specific waypoint."""
        dx = waypoint[0] - current[0]
        dy = waypoint[1] - current[1]
        angle_to_goal = math.atan2(dy, dx)
        distance_to_goal = math.sqrt(dx**2 + dy**2)

        self.rotate_to(angle_to_goal)
        self.forward(distance_to_goal)

    def control_loop(self):
        self.wait_for_slam()
        path = []

        while not rospy.is_shutdown():
            if self.map_resolution is None or self.map_origin_x is None or self.map_origin_y is None:
                rospy.logwarn("Map parameters not initialized. Waiting for map data...")
                self.rate.sleep()
                continue

            goal_pixel_x = int((self.goal.x - self.map_origin_x) / self.map_resolution)
            goal_pixel_y = int((self.goal.y - self.map_origin_y) / self.map_resolution)

            robot_pixel_x = int((self.pose.x - self.map_origin_x) / self.map_resolution)
            robot_pixel_y = int((self.pose.y - self.map_origin_y) / self.map_resolution)

            if (robot_pixel_x, robot_pixel_y) == (goal_pixel_x, goal_pixel_y):
                rospy.loginfo("End goal reached!")
                self.update_command_velocity(0.0, 0.0)
                break

            if not path or (robot_pixel_x, robot_pixel_y) == path[0]:
                path = self.a_star((robot_pixel_x, robot_pixel_y), (goal_pixel_x, goal_pixel_y))
                if not path:
                    rospy.logwarn("No valid path found to goal. Retrying...")
                    self.rate.sleep()
                    continue

            next_pixel = path[0]
            next_x = self.map_origin_x + next_pixel[0] * self.map_resolution
            next_y = self.map_origin_y + next_pixel[1] * self.map_resolution

            self.move((self.pose.x, self.pose.y), (next_x, next_y))

            if (robot_pixel_x, robot_pixel_y) == path[0]:
                rospy.loginfo(f"Intermediate goal {path[0]} reached.")
                path.pop(0)

            self.rate.sleep()

    def wait_for_slam(self):
        rospy.loginfo("Waiting for SLAM to start...")
        while not rospy.is_shutdown():
            if self.map_data is not None and self.costmap_2d is not None:
                rospy.loginfo("SLAM started, map and costmap initialized.")
                return
            rospy.sleep(1)

    def a_star(self, start, goal):
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                return self.reconstruct_path(came_from, current)

            for neighbor in self.get_neighbors(current):
                tentative_g_score = g_score[current] + 1
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return []

    def heuristic(self, neighbor, goal):
        return abs(neighbor[0] - goal[0]) + abs(neighbor[1] - goal[1])

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def get_neighbors(self, position):
        x, y = position
        neighbors = []
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nx, ny = x + dx, y + dy
            if 0 <= nx < len(self.costmap_2d[0]) and 0 <= ny < len(self.costmap_2d) and self.costmap_2d[ny][nx] != 100:
                neighbors.append((nx, ny))
        return neighbors


if __name__ == '__main__':
    try:
        turtlebot3_drive = Turtlebot3Drive()
        turtlebot3_drive.control_loop()
    except rospy.ROSInterruptException:
        pass
