#!/usr/bin/env python3

import rospy
import math
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseArray
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from ellipse_movement import EllipseMovement
from geometry_msgs.msg import Pose, PoseArray

class PathFollower:
    def __init__(self):
        rospy.init_node('controller')
        
        # Controller parameters
        self.linear_speed = 0.2  # meters per second
        self.angular_speed = 0.4  # radians per second
        self.distance_threshold = 0.03  # meters
        self.angle_threshold = 0.2  # radians about 11 grades
        self.obstacle_timer = None
        
        
        # Collision avoidance parameters
        self.dangerous_distance = 0.14  # meters
        self.scan_angles = {
            'front': (-20, 20),  # degrees
            'front_left': (20, 40),
            'front_right': (320, 340),
            'left': (60, 100),
            'right': (260, 300)
        }
        self.movement_lock = False
        
        # Robot state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.path = []
        self.current_goal_index = 0
        self.save_path_once = False
        self.distances = {}  # Store distances to walls in different directions
        self.avoiding_obstacle = False
        self.goalX = 0
        self.goalY = 0
        self.new_goal_x = 0
        self.new_goal_y = 0
        self.new_end_point = False


        # Publishers and Subscribers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.path_sub = rospy.Subscriber('/planned_path', PoseArray, self.path_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        # TF listener for coordinate transformations
        self.tf_listener = tf.TransformListener()
        
        rospy.loginfo("Turtlebot3 Controller initialized")


    def scan_callback(self, msg):
        """Process LiDAR scan data to detect walls and obstacles."""
        # Convert angles from degrees to array indices
        angle_increment = msg.angle_increment
        num_readings = len(msg.ranges)

        # Process each direction defined in scan_angles
        for direction, (start_deg, end_deg) in self.scan_angles.items():
            # Convert degrees to radians
            start_rad = math.radians(start_deg)
            end_rad = math.radians(end_deg)
            
            # Convert radians to array indices
            start_idx = int((start_rad - msg.angle_min) / angle_increment)
            end_idx = int((end_rad - msg.angle_min) / angle_increment)
            
            # Ensure indices are within bounds
            start_idx = max(0, min(start_idx, num_readings - 1))
            end_idx = max(0, min(end_idx, num_readings - 1))
            
            # Get relevant ranges for this direction
            ranges = msg.ranges[start_idx:end_idx]
            
            # Filter out invalid readings
            valid_ranges = [r for r in ranges if msg.range_min <= r <= msg.range_max]
            #rospy.loginfo(f"Right Angles: Start index: {start_idx}, End index: {end_idx}, Ranges: {ranges}")

            
            if valid_ranges:
                # Store the minimum distance for this direction
                self.distances[direction] = min(valid_ranges)
            else:
                # Log warning if no valid readings for this direction
                rospy.logwarn(f"No valid readings for direction {direction}")
                # Assume maximum range
                self.distances[direction] = msg.range_max

        
        # Check for obstacle and adjust path if needed
        #rospy.loginfo(f"Distances to walls: {self.distances}")
        self.adjust_path_for_obstacle()

    def adjust_path_for_obstacle(self):
        """Dynamically adjust the current path point to avoid obstacles."""
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0

        if not self.path or self.current_goal_index >= len(self.path):
            return cmd_vel
        
        # Check if it is too close
        front_dist = self.distances.get('front', float('inf')) #float infinity ,used as default value if the key does not exists it is a safe default value
        left_dist = self.distances.get('left', float('inf'))
        right_dist = self.distances.get('right', float('inf'))
        front_right_dist = self.distances.get('front_right', float('inf'))
        front_left_dist = self.distances.get('front_left', float('inf'))

        # State variable to track obstacle avoidance state
        if not hasattr(self, 'obstacle_state'):
            self.obstacle_state = 'no_obstacle'
            self.obstacle_timer = rospy.Time.now()
            rospy.loginfo("no_obstacle")

        #rospy.loginfo(f"front: {front_dist}, left: {left_dist}, right: {right_dist}")
        #rospy.loginfo(f"front_left: {front_left_dist}, front_right: {front_right_dist}")

        '''
        goal_x, goal_y = self.path[self.current_goal_index]
        if front_dist < self.get_distance_to_goal(goal_x, goal_y) and abs(self.get_angle_to_goal(goal_x, goal_y)) < self.angle_threshold: # then there is an obstacle between the robot and the next goal
            self.movement_lock = True
            self.obstacle_state = 'obstacle_observed'
            prev_x,prev_y = self.path[self.current_goal_index-2] # get the prev point
            cmd_vel.angular.z = self.angular_speed if self.get_angle_to_goal(prev_x,prev_y) < 0 else -self.angular_speed  #rotate to the opposite diraction of the last point
            cmd_vel.linear.x = 0.0
            rospy.loginfo(f"obstacle between, dis: {self.get_distance_to_goal(goal_x, goal_y)}, angle: {self.get_angle_to_goal(prev_x, prev_y)}")
            rospy.loginfo(f"next goal : {self.current_goal_index}")
            self.cmd_vel_pub.publish(cmd_vel)

            if abs(self.get_angle_to_goal(prev_x, prev_y)) > self.angle_threshold: # the rotation is completed
                self.obstacle_timer = rospy.Time.now()
                while((rospy.Time.now() - self.obstacle_timer).to_sec() < 1): #move forward and move to the next goal
                    cmd_vel.linear.x = 0.2
                    self.cmd_vel_pub.publish(cmd_vel)
                self.current_goal_index += 1
                self.movement_lock = False
        ''' 
        if front_dist < self.dangerous_distance or front_right_dist < self.dangerous_distance or front_left_dist < self.dangerous_distance:
            #rospy.loginfo("in!!!!!!!!!")
            self.obstacle_timer = rospy.Time.now()
            self.movement_lock = True
            while((rospy.Time.now() - self.obstacle_timer).to_sec() < 0.1):
                cmd_vel.linear.x = -0.1
                self.cmd_vel_pub.publish(cmd_vel)

            if self.obstacle_state == 'no_obstacle':
                self.obstacle_state = 'rotating'
                rospy.loginfo("rotating")
                cmd_vel.linear.x = 0
                # Decide rotation direction based on which side has more space
                if self.get_angle_to_goal(self.goalX, self.goalY) < 0:  
                    while ((rospy.Time.now() - self.obstacle_timer).to_sec() < 2):
                        #rospy.loginfo((rospy.Time.now() - self.obstacle_timer).to_sec())              
                        cmd_vel.angular.z = 0.2  # Rotate left
                        self.cmd_vel_pub.publish(cmd_vel)
                else:
                    while ((rospy.Time.now() - self.obstacle_timer).to_sec() < 2): 
                        cmd_vel.angular.z = -0.2  # Rotate right
                        self.cmd_vel_pub.publish(cmd_vel)

            # After rotation, move forward
            elif self.obstacle_state == 'moving_forward':
                rospy.loginfo("moving_forward 2")
                cmd_vel.linear.x = self.linear_speed
                cmd_vel.angular.z = 0
                self.movement_lock = False

        # Check if we can switch to moving forward
        if self.obstacle_state == 'rotating':
            # Stop rotating and prepare to move forward
            self.obstacle_state = 'moving_forward'
            rospy.loginfo("moving_forward 1")
            cmd_vel.linear.x = self.linear_speed
            cmd_vel.angular.z = 0
        
        self.cmd_vel_pub.publish(cmd_vel)
        
    def odom_callback(self, msg):
        """Update robot's current position and orientation from odometry."""
        self.current_x= self.calculate_real_world_position(msg.pose.pose.position.x)
        self.current_y= self.calculate_real_world_position(msg.pose.pose.position.y)

        # Convert quaternion to Euler angles
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, self.current_yaw = euler_from_quaternion(orientation_list)

    def path_callback(self, msg):
        """Receive and store the planned path."""
        if not self.save_path_once:
            self.save_path_once= True
            self.path = [(pose.position.x, pose.position.y) for pose in msg.poses]
            self.current_goal_index = 1

    def get_distance_to_goal(self, goal_x, goal_y):
        """Calculate Euclidean distance to the goal."""
        return math.sqrt((goal_x - self.current_x)**2 + (goal_y - self.current_y)**2)/100

    def get_angle_to_goal(self, goal_x, goal_y):
        """Calculate angle to the goal relative to robot's current heading."""
        goal_angle = math.atan2(goal_y - self.current_y, goal_x - self.current_x)
        angle_diff = goal_angle - self.current_yaw
        
        # Normalize angle to [-pi, pi]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
            
        return angle_diff

    def calculate_real_world_position(self,robot_pose):
        position = int((robot_pose - (-10.0)) / 0.025) #-10 is map_origin , and the 0.025 is the map resolution
        return position

    def add_help_point(self,goal_x,goal_y):
        prev_x_1,prev_y_1 = self.path[self.current_goal_index-1] # get the prev point
        prev_x_2,prev_y_2 = self.path[self.current_goal_index-2]
        rospy.loginfo(f"in the first if, angle to goal: {self.get_angle_to_goal(goal_x, goal_y)}")
        if prev_x_1 == prev_x_2:
            self.new_goal_x = prev_x_1
            rospy.loginfo("in the x = if")
            if prev_y_1 > prev_y_2:
                _,self.new_goal_y = self.path[self.current_goal_index]
                self.new_goal_y += 1
            else:
                _,self.new_goal_y = self.path[self.current_goal_index]
                self.new_goal_y -= 1
        else:
            self.new_goal_y = prev_y_1
            rospy.loginfo("in the y = if")
            if prev_x_1 > prev_x_2:
                self.new_goal_x,_ = self.path[self.current_goal_index]
                self.new_goal_x += 1
            else:
                self.new_goal_x,_ = self.path[self.current_goal_index]
                self.new_goal_x -= 1
        rospy.loginfo(f"x: {goal_x}, y: {goal_y}")

    def move_to_goal(self):
        """Generate velocity commands to move towards the current goal."""
        if not self.path or self.current_goal_index >= len(self.path):
            return self.stop_robot()
        '''   
        if self.new_goal_x == 0 and self.new_goal_y == 0:
            # Get current goal position
            goal_x, goal_y = self.path[self.current_goal_index]
            self.goalX = goal_x
            self.goalY = goal_y
        else:
            goal_x = self.new_goal_x
            goal_y = self.new_goal_y
        '''
        goal_x, goal_y = self.path[self.current_goal_index]
        self.goalX = goal_x
        self.goalY = goal_y
        # Calculate distance and angle to goal
        distance_to_goal = self.get_distance_to_goal(goal_x, goal_y)
        angle = self.get_angle_to_goal(goal_x, goal_y)

        #rospy.loginfo(f"distance: {distance}, angle_diff: {angle}")
        #rospy.loginfo(f"goal_x: {goal_x}, goal_y: {goal_y}")
        #rospy.loginfo(f"current_x: {self.current_x}, current_y: {self.current_y}")
        #rospy.loginfo(f"current_yaw: {self.current_yaw}, goal_angle: {math.atan2(goal_y - self.current_y, goal_x - self.current_x)}")
            
        
        cmd_vel = Twist()
        
        self.check_line(goal_x,goal_y) #get the last point in the line (not point by point)
        goal_x, goal_y = self.path[self.current_goal_index]
        rospy.loginfo(f"x:{goal_x}, y:{goal_y}, i:{self.current_goal_index}")

        if abs(angle) > self.angle_threshold:
            cmd_vel.angular.z = self.angular_speed if angle > 0 else -self.angular_speed
            cmd_vel.linear.x = 0.0
            '''
            if self.distances.get('front', float('inf')) < self.get_distance_to_goal(goal_x, goal_y) and self.current_goal_index >1 and self.new_end_point == False: # then there is an obstacle between the robot and the next goal
                self.add_help_point(goal_x,goal_y)
                self.new_end_point = True
                rospy.loginfo(f"distance {self.get_distance_to_goal(goal_x, goal_y)}, front: {self.distances.get('front', float('inf'))}, index: {self.current_goal_index}")
            rospy.loginfo("im in the move to goal")
            '''
        else :
            cmd_vel.linear.x = self.linear_speed
            cmd_vel.angular.z = 0.0
        
            # Check if we've reached the current waypoint
        if distance_to_goal < self.distance_threshold:
            '''
            if self.new_goal_x != 0 and self.new_goal_y != 0: # if the point is edited to avoid the corner
                self.new_goal_x = 0
                self.new_goal_y = 0
                self.new_end_point = False
                rospy.loginfo("new goal reset")
            else:
            '''
            self.current_goal_index += 1
            rospy.loginfo(f"Reached waypoint {self.current_goal_index-1} of {len(self.path)}")
            if self.current_goal_index >= len(self.path):
                rospy.loginfo("Reached final goal!")
                return self.stop_robot()

        return cmd_vel

    def check_line(self,goal_x,goal_y):
        if self.current_goal_index+1 != len(self.path):
            counter = 1
            next_goal_x, next_goal_y = self.path[self.current_goal_index + counter]

            while next_goal_x == goal_x:
                counter += 1
                if self.current_goal_index + counter < len(self.path):
                    #rospy.loginfo(f"in!!!!!!!!!!!")
                    next_goal_x,_ = self.path[self.current_goal_index + counter]
                    if next_goal_x != goal_x:
                        #rospy.loginfo(f"goal_x: {goal_x}, goal_y: {goal_y}, index : {self.current_goal_index} from x")
                        self.current_goal_index = max(self.current_goal_index + counter -3,self.current_goal_index)
                        return
                else:
                    self.current_goal_index = max(self.current_goal_index + counter -3,self.current_goal_index)
                    return
                    
            counter = 1
            while next_goal_y == goal_y:
                counter += 1
                #rospy.loginfo(f"counter: {counter}")
                if self.current_goal_index + counter < len(self.path):
                    #rospy.loginfo(f"in2222222222")
                    _,next_goal_y = self.path[self.current_goal_index + counter]
                    if next_goal_y != goal_y:
                        self.current_goal_index = max(self.current_goal_index + counter -3,self.current_goal_index)
                        #rospy.loginfo(f"next_goal_y: {next_goal_y}, goal_y: {goal_y}, index : {self.current_goal_index} from y")
                        return
                else:
                    self.current_goal_index = max(self.current_goal_index + counter -3,self.current_goal_index)
                    return
        return

    def stop_robot(self):
        """Generate velocity command to stop the robot."""
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        return cmd_vel

    def run(self):
        """Main control loop."""
        rate = rospy.Rate(10)  # 10 Hz
        
        while not rospy.is_shutdown():
            # Generate and publish velocity command
            if not self.movement_lock:
                cmd_vel = self.move_to_goal()
            self.cmd_vel_pub.publish(cmd_vel)
            rate.sleep()
            '''
            cmd_vel = Twist()  # Default to no movement
            if self.movement_lock:
                cmd_vel = self.adjust_path_for_obstacle()
            else:
                cmd_vel = self.move_to_goal()
            self.cmd_vel_pub.publish(cmd_vel)
            rate.sleep()
            '''

if __name__ == '__main__':
    try:
        controller = PathFollower()
        controller.run()
    except rospy.ROSInterruptException:
        pass