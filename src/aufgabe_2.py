#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist, Point, PoseArray
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from tf.transformations import euler_from_quaternion
from collections import deque
from turtlebot_maze_navigation.msg import MapData

"""
The Node class aggregates the coordinates of a point 
and the coordinates of the neighboring points (not neighboring Nodes!!!)
"""
class Node:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.parent = None #Node
        self.right = None #Node
        self.left = None #Node
        self.front = None #Node
        self.visited = False 
        
    def __eq__(self, other):
        if not isinstance(other, Node):
            return False  # Wenn `other` keine Instanz von `Node` ist, ist es nicht gleich
        return self.x == other.x and self.y == other.y

    
    #def get_distance_to_temp_goal(self, pos_x, pos_y):
        #"""Calculate Euclidean distance to the goal."""
        #return math.sqrt((pos_x - self.x)**2 + (pos_y - self.y)**2)
    
    #def in_range(self, other):
        #return self.get_distance_to_temp_goal(other.x, other.y) <= 0.3 # Meter

    #def __hash__(self):
        #return hash(self.position)
   


# Konstanten für Umrechnung und Geschwindigkeit
class Turtlebot3Explorer:
    def __init__(self):
        rospy.init_node('aufgabe2', anonymous=True)
        rospy.loginfo("aufgabe_2 initialized")

        # SLAM und Kartierungsvariablen
        self.return_to_x = []  # Store X coordinates of blocked dead-ends
        self.return_to_y = []  # Store Y coordinates of blocked dead-ends
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.current_x_real = 0.03374290466308594
        self.current_y_real = -0.0013123006792739
        
        self.start_x = None
        self.start_y = None
        self.goal_x = 3.6837387084960938
        self.goal_y = -3.395461082458496 
        self.linear_speed = 0.1 #meters
        self.laser_scan_msg = 0

        self.curr_node = None
        self.curr_dir = 0
        self.curr_parent = None

        self.map = None
        self.map_resolution = None
        self.map_origin_x = None
        self.map_origin_y = None
        self.map_width = None
        self.map_height = None
        self.costmap_2d = None
        self.distance_threshold = 0.01  # meters

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
        self.distances = {}  # Store distances to walls in different directions

        # Publisher und Subscriber
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.position_pub = rospy.Publisher('/robot_path', Point, queue_size=10)
        self.temp_start_pub = rospy.Publisher('/temp_start_point', Point, queue_size=10)
        self.temp_goal_pub = rospy.Publisher('/temp_goal_point', Point, queue_size=10)
        self.temp_map_pub = rospy.Publisher('/temp_map_data', MapData, queue_size=10)
        
        self.scan_sub = rospy.Subscriber("scan", LaserScan, self.laser_scan_callback)
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)
        self.map_sub = rospy.Subscriber("map", OccupancyGrid, self.map_callback)

    def odom_callback(self, msg: Odometry):
        """Update robot's current position and orientation from odometry."""
        self.current_x_real=-msg.pose.pose.position.x-0.75
        self.current_y_real=-msg.pose.pose.position.y+0.75
        self.current_yaw= msg.pose.pose.position.z

        if self.curr_node is None:
            node = Node()
            node.x = self.current_x_real
            node.y = self.current_y_real
            node.visited = True
            self.curr_node = node
            self.curr_parent = node

        # Extrahiere Orientierung des Roboters
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]

        # Konvertiere orientation_list in Euler-Winkel (Roll, Pitch, Yaw)
        _, _, self.current_yaw = euler_from_quaternion(orientation_list)

    def calculate_real_world_position(self,robot_pose):
        position = int((robot_pose - (-10.0)) / 0.025) #-10 is map_origin , and the 0.025 is the map resolution
        return position

    def get_angle_to_goal(self, goal_x, goal_y):
        """Calculate angle to the goal relative to robot's current heading."""
        dy = goal_y - self.current_y_real
        dx = goal_x - self.current_x_real
        
        # Invert the y-coordinates if your coordinate system has y increasing downwards
        goal_angle = math.atan2(dy, dx)
        angle_diff = goal_angle - self.current_yaw
        angle_diff_2 = (angle_diff) % (2 * math.pi) - math.pi
        
        return angle_diff_2
    
    def map_callback(self, msg: OccupancyGrid):

        if not msg.data or len(msg.data) == 0:
            rospy.logwarn("Leere oder ungültige SLAM-Daten empfangen. map_callback wird nicht ausgeführt.")
            return

        
    def laser_scan_callback(self, msg):
        """Process LiDAR scan data to detect walls and obstacles."""
        # Convert angles from degrees to array indices
        self.laser_scan_msg = msg
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

        #if self.check_closed_way(msg):
            #self.return_to_the_last_crossroads()


    def get_distance_to_temp_goal(self, goal_x, goal_y):
        """Calculate Euclidean distance to the goal."""
        return math.sqrt((goal_x - self.current_x_real)**2 + (goal_y - self.current_y_real)**2)
    
    def log_all_nodes(self, node, depth=0):
        """
        Rekursive Funktion, um alle Nodes auszugeben.
        Args:
            node: Die aktuelle Node, von der aus iteriert wird.
            depth: Die aktuelle Tiefe, um die Hierarchie zu verdeutlichen.
        """
        if node is None:
            return
        
        # Logge die aktuelle Node mit Einrückung basierend auf der Tiefe
        rospy.logwarn(f"{'  ' * depth}Node at depth {depth}: x={node.x}, y={node.y}, visited={node.visited}")
        
        # Rufe die Funktion rekursiv für alle verbundenen Nodes auf
        if node.left:
            rospy.logwarn(f"{'  ' * depth}Left:")
            self.log_all_nodes(node.left, depth + 1)
        if node.right:
            rospy.logwarn(f"{'  ' * depth}Right:")
            self.log_all_nodes(node.right, depth + 1)
        if node.front:
            rospy.logwarn(f"{'  ' * depth}Front:")
            self.log_all_nodes(node.front, depth + 1)

    
    
    def move(self):
        cmd_vel = Twist()
        front_dist = self.distances.get('front', float('inf'))
        left_dist = self.distances.get('left', float('inf'))
        right_dist = self.distances.get('right', float('inf'))

        
        if  10 > left_dist > 0.4 or 10 > right_dist > 0.4: # add : add distance to current Node is not smaller that 0.5, because 0.5 is the width of the tunle
            '''
            father erstellt
            an dem father hinzufügen
            current node moved to the front node (aktuelle Node)
            '''


            #self.curr_parent = self.curr_node.parent
            #if self.get_distance_to_temp_goal(self.curr_node.x, self.curr_node.y) > 0.3:
            if self.curr_node.left is not None:
                self.curr_node.left.x, self.curr_node.left.y = self.current_x_real, self.current_y_real
                self.curr_node.left.visited = True
                self.curr_node = self.curr_node.left
                rospy.loginfo("Zu linker Node gewechselt:")
            elif self.curr_node.right is not None:
                rospy.loginfo(f"Aktuelle Position: x={self.current_x_real}, y={self.current_y_real}") # aktuelle roboterposition korrekt
                self.curr_node.right.x, self.curr_node.right.y = self.current_x_real, self.current_y_real # wird aber hier nicht aktualisiert (Wieso?)
                self.curr_node.right.visited = True
                self.curr_node = self.curr_node.right
                rospy.loginfo("Zu rechter Node gewechselt:")
            elif self.curr_node.front is not None:
                self.curr_node.front.x, self.curr_node.front.y = self.current_x_real, self.current_y_real
                self.curr_node.front.visited = True
                self.curr_node = self.curr_node.right
                rospy.loginfo(f"Zu front Node gewechselt:")
            else:
                node = Node() 
                node.x, node.y = self.current_x_real, self.current_y_real
                node.visited = True
                node.parent = self.curr_parent 
                self.curr_node = node
                rospy.loginfo(f"Neue Node erstellt:")

            """
            if self.curr_dir != None:
                if self.curr_dir == 1:
                    self.curr_node.left.x, self.curr_node.left.y = node.x, node.y
                elif self.curr_dir == -1:
                    self.curr_node.right.x, self.curr_node.right.y = node.x, node.y
                else:
                    self.curr_node.front.x, self.curr_node.front.y = node.x, node.y
            """       
            
            

            cmd_vel.linear.x = 0
            direction = -1 #self.get_the_new_orientation() #the direction is used after 3 lines to define the direction of the rotation
            self.curr_dir = direction
        
            if direction == 1:
                left = Node()
                left.parent = self.curr_node
                self.curr_node.left = left 
            elif direction == -1:
                right = Node()
                right.parent = self.curr_node
                self.curr_node.right = right
            else:
                front = Node()
                front.parent = self.curr_node
                self.curr_node.front = front

            self.curr_node.parent = self.curr_node
            self.curr_parent = self.curr_node

            rospy.loginfo("Protokolliere alle Nodes:")
            self.log_all_nodes(self.curr_node)

            #if direction != 0:
                #while abs(self.current_yaw) - (direction* math.pi/2) > 0.01:
                    #cmd_vel.angular.z =  direction*0.4
                    #self.cmd_vel_pub.publish(cmd_vel)
            #rospy.loginfo(f"yaw: {self.current_yaw}, pi: {math.pi/2}, z {cmd_vel.angular.z}")
            

        else: 
            #rospy.loginfo(f" left : {left_dist}, right {right_dist}")
            #cmd_vel.angular.z = 0
            #cmd_vel.linear.x = 0.4
            #rospy.loginfo(f"speed: {cmd_vel.linear.x}, distnace: {front_dist}")
            
            #self.cmd_vel_pub.publish(cmd_vel)
            
            
            '''
            self.correct_the_position(left_dist,right_dist)

            if self.laser_scan_msg != 0:
                if self.check_closed_way():
                    # Bedingung muss angepasst werden return_to_the_last_crossroads() soll nur aufgerufen werden, wenn es eine Sackgasse gibt 
                    # bzw. der Robotererkennt, dass der Pfad ihn nicht weiterführt, weil er diesen schon abgeklappert hat 
                    self.return_to_the_last_crossroads()
            '''
          
           
    '''
    #return +1 (against the O'clock) or -1
    get the direction (left, right) based on , which one is closer to the goal
    must be checked if the left / right open and there is no wall there.
    #you have also to check if this way is already visited !!!
    the best way is to save the points in the move() function after the cmd_vel.linear.x to make sure that you have saved all visited points
    '''
    #Sirine
    def get_the_new_orientation(self):
        """
        Determine the new orientation at a crossroads to move closer to the goal.
    
        Returns:
            +1: Turn left
            -1: Turn right 
            0: Move forward
        """
        left_dist = self.distances.get('left', float('inf'))
        right_dist = self.distances.get('right', float('inf'))

        # Check if paths are clear
        left_clear = left_dist > 0.4
        right_clear = right_dist > 0.4


        if right_clear:
            #self.goal_yaw = 
            return -1
        elif left_clear:
            return 1
        else:
            return 0

    '''
    go back to the last point where you were before you made the dissicion to turn left or right
    this point is saved already
    maybe you have to go two times back 
    (so if the robot arrived to the point you have to delete it from the arrays (return_to_x and return_to_y) 
        in order to make the last new point is the point where the robot can return to it)
    you can use in this function the a_star algorithm from Part 1 and set the last item in the return_to_x and return_to_y arrays as the goal item
    and use the calulate distance function to check if you arrived to the goal
    '''
    #Soufian
    def return_to_the_last_crossroads(self):
        if self.curr_node is None:
            rospy.logwarn("Node besitzt kein Parent")
            return
        goal_x, goal_y = self.curr_node.parent.x, self.curr_node.parent.y
        self.move_to_goal(goal_x, goal_y)



    '''
    return true if the road closed, false if not
    it has to check it using the laser data (if there is no a x_distance between the points of the laser then it is closed)
    the x_distance is smaller a little bit than the spaces between the walls (it needs to be tested and adjasted rightly)
    '''
    def check_closed_way(self):
        """
        Check if the path is closed by analyzing gaps between sequential laser scan points.
        Returns True if all directions (front, left, right) are closed with no sufficient gaps.
        """
        if not self.laser_scan_msg:
            return False

        MIN_PASSAGE_WIDTH = 0.29  # Minimum width needed for robot to pass through
        MAX_RANGE_CHECK = 3.0     # Maximum distance to check for obstacles
        
        # Define angle ranges for each direction (in degrees)
        #set a shared angle between the directions like 35 left and 40 right to prevent not cosidering of the points at the edges
        ANGLE_RANGES = {
            'front': [(320, 360), (0, 40)],    # front range
            'left': [(35, 110)],               # left range
            'right': [(250, 325)]              # right range
        }
        
        angle_min = self.laser_scan_msg.angle_min
        angle_increment = self.laser_scan_msg.angle_increment
        ranges = self.laser_scan_msg.ranges
        
        def analyze_direction(angle_range):
            """Helper function to analyze distances in a specific angle range"""
            start_deg, end_deg = angle_range
            
            # Convert degrees to indices
            start_idx = int((math.radians(start_deg) - angle_min) / angle_increment)
            end_idx = int((math.radians(end_deg) - angle_min) / angle_increment)
            
            # Ensure indices are within bounds
            start_idx = max(0, min(start_idx, len(ranges) - 1))
            end_idx = max(0, min(end_idx, len(ranges) - 1))
            
            valid_readings = []
            for i in range(start_idx, end_idx):
                range_val = ranges[i]
                if self.laser_scan_msg.range_min <= range_val <= MAX_RANGE_CHECK:
                    valid_readings.append(range_val)
            
            if not valid_readings:
                return True  # Consider it closed if no valid readings
                
            # Find the maximum gap between any two readings
            max_gap = 0
            for i in range(len(valid_readings) - 1):
                gap = abs(valid_readings[i] - valid_readings[i + 1])
                max_gap = max(max_gap, gap)
                
            #rospy.loginfo(f"Max gap in range {start_deg}-{end_deg}: {max_gap:.3f}m")
            return max_gap < MIN_PASSAGE_WIDTH
        
        #Check each direction
        for _, angle_ranges in ANGLE_RANGES.items():
            for angle_range in angle_ranges:
                if not analyze_direction(angle_range):
                    #rospy.loginfo(f"Passage found in {direction} direction")
                    return False
        
        rospy.loginfo("All directions appear closed")
        return True
    

    def move_to_goal(self,goal_x,goal_y):

        # Calculate distance and angle to goal
        distance_to_goal = self.get_distance_to_temp_goal(goal_x, goal_y)
        angle_diff = self.get_angle_to_goal(goal_x, goal_y)
        
        cmd_vel = Twist()
        
        while (distance_to_goal < 0.1):
            if abs(angle_diff) > 0.1:
                cmd_vel.linear.x = 0.0
                if abs(angle_diff) > 0.4:
                    cmd_vel.angular.z = 0.6 if angle_diff > 0 else -0.6
                elif abs(angle_diff) > 0.2:
                    cmd_vel.angular.z = 0.3 if angle_diff > 0 else -0.3
                elif abs(angle_diff) > 0.1:
                    cmd_vel.angular.z = 0.2 if angle_diff > 0 else -0.2
            else:
                cmd_vel.angular.z = 0.0
                if distance_to_goal > 1:
                    cmd_vel.linear.x = 0.5
                elif distance_to_goal > 0.5:
                    cmd_vel.linear.x = 0.4
                elif distance_to_goal > 0.3:
                    cmd_vel.linear.x = 0.2
                elif distance_to_goal > 0.1:
                    cmd_vel.linear.x = 0.15
            
            self.cmd_vel_pub.publish(cmd_vel)
        
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)

        
    
    '''
    return: publish the corrected position
    goal: try to keep the robot in the middle between the walls
    you can use the left_dist and right_dist with a badding about 0.5 meters
    so not every time we need to correct the position
    '''
    #Danny
    def correct_the_position(self,left_dist,right_dist):
        cmd_vel = Twist()
        #rospy.loginfo(f"left: {left_dist}, right: {right_dist}")
        CORRECTION_THRESHOLD = 0.1  # Minimum difference to trigger correction
        
        # Calculate the difference between left and right distances
        difference = left_dist - right_dist
        
        if abs(difference) > CORRECTION_THRESHOLD:
            # Calculate correction factor (between -1 and 1)
            correction = max(min(difference / (left_dist + right_dist), 1.0), -1.0)
            
            # Apply proportional correction
            cmd_vel.angular.z = -correction * 0.2  # Negative because right turn needs negative angular velocity
            cmd_vel.linear.x = 0.1 * (1 - abs(correction))  # Slow down during correction
            
            rospy.loginfo(f"Correcting position: diff={difference:.2f}, correction={correction:.2f}")
        else:
            # No correction needed, move forward
            cmd_vel.linear.x = 0.1
            cmd_vel.angular.z = 0.0
        
        # Publish the command
        self.cmd_vel_pub.publish(cmd_vel)
            

    def control_loop(self):
        # Hauptkontrollschleife
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.get_distance_to_temp_goal(self.goal_x,self.goal_y) < 0.1:
                rospy.loginfo("goal reached")
                self.stop_robot()
                break
            self.move()
            rospy.sleep(0.1)  # Pause for smooth data processing
            rate.sleep()

    def stop_robot(self):
        """Generate velocity command to stop the robot."""
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        return cmd_vel

if __name__ == '__main__':
    try:
        explorer = Turtlebot3Explorer()
        explorer.control_loop()
        #rospy.spin()  #to keep the node running
    except rospy.ROSInterruptException as e:
        rospy.loginfo(e)