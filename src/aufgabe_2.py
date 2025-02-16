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
        self.node_1 = None #Node
        self.node_2 = None #Node
        self.node_3 = None #Node
        self.visited = False
        self.number_of_roads = 0
        
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

        self.current_x_real = 0.03374290466308594
        self.current_y_real = -0.0013123006792739
        self.current_yaw = 0.0
        
        self.goal_x = 3.6837387084960938
        self.goal_y = -3.395461082458496
        self.laser_scan_msg = 0
        self.temp_goal_x = 0
        self.temp_goal_y = 0
        #self.last_direction = 0

        self.curr_node = None
        self.curr_dir = 0
        self.curr_parent = None
        self.to_direction_up_down = None

        self.map = None
        self.called = False

        #wurzel erstellen
        if self.curr_node is None:
            node = Node()
            node.x = self.current_x_real
            node.y = self.current_y_real
            node.visited = True
            self.curr_node = node
            #self.curr_node.parent = node #the root is the parent of itself
            self.curr_parent = node
            self.curr_node.number_of_roads = 2 #need to make this dinamic

        self.move_beside_left = True
        self.distances = {}  # Store distances to walls in different directions
        self.distances_min = {}

        # Publisher und Subscriber
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        self.scan_sub = rospy.Subscriber("scan", LaserScan, self.laser_scan_callback)
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)
        self.map_sub = rospy.Subscriber("map", OccupancyGrid, self.map_callback)

    def odom_callback(self, msg: Odometry):
        """Update robot's current position and orientation from odometry."""
        self.current_x_real= -msg.pose.pose.position.x - 0.75
        self.current_y_real= -msg.pose.pose.position.y + 0.75

        # Extrahiere Orientierung des Roboters
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]

        # Konvertiere orientation_list in Euler-Winkel (Roll, Pitch, Yaw)
        _, _, self.current_yaw = euler_from_quaternion(orientation_list)

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

        # Calculate indices for specific angles
        front_idx = int((0 - msg.angle_min) / angle_increment)
        left_idx = int((math.radians(90) - msg.angle_min) / angle_increment)
        left_back_idx = int((math.radians(102) - msg.angle_min) / angle_increment)
        #left_front_idx = int((math.radians(85) - msg.angle_min) / angle_increment)

        right_idx = int((math.radians(-90) - msg.angle_min) / angle_increment)
        right_back_idx = int((math.radians(-102) - msg.angle_min) / angle_increment)
        #right_front_idx = int((math.radians(-85) - msg.angle_min) / angle_increment)


        # Calculate indices for 60 and 90 degrees
        left_25_idx = int((math.radians(25) - msg.angle_min) / angle_increment)
        left_90_idx = int((math.radians(90) - msg.angle_min) / angle_increment)
        
        # Get minimum distance in the 60-90 degree range
        left_25_90_range = msg.ranges[left_25_idx:left_90_idx + 1]
        # Filter out invalid readings
        valid_left_readings = [r for r in left_25_90_range if msg.range_min <= r <= msg.range_max]
        min_left_25_90 = min(valid_left_readings) if valid_left_readings else msg.range_max

        # Calculate indices for 60 and 90 degrees
        right_25_idx = int((math.radians(-25) - msg.angle_min) / angle_increment)
        right_90_idx = int((math.radians(-90) - msg.angle_min) / angle_increment)
        
        # Get minimum distance in the 60-90 degree range
        right_25_90_range = msg.ranges[right_90_idx:right_25_idx + 1]
        # Filter out invalid readings
        valid_right_readings = [r for r in right_25_90_range if msg.range_min <= r <= msg.range_max]
        min_right_25_90 = min(valid_right_readings) if valid_right_readings else msg.range_max

        # Get readings
        self.distances = {
            'front': msg.ranges[front_idx] if msg.range_min <= msg.ranges[front_idx] <= msg.range_max else msg.range_max,

            'left': msg.ranges[left_idx] if msg.range_min <= msg.ranges[left_idx] <= msg.range_max else msg.range_max,
            'left_back': msg.ranges[left_back_idx] if msg.range_min <= msg.ranges[left_back_idx] <= msg.range_max else msg.range_max,
            #'left_front': msg.ranges[left_front_idx] if msg.range_min <= msg.ranges[left_front_idx] <= msg.range_max else msg.range_max,

            'right': msg.ranges[right_idx] if msg.range_min <= msg.ranges[right_idx] <= msg.range_max else msg.range_max,
            'right_back': msg.ranges[right_back_idx] if msg.range_min <= msg.ranges[right_back_idx] <= msg.range_max else msg.range_max,
            #'right_front': msg.ranges[right_front_idx] if msg.range_min <= msg.ranges[right_front_idx] <= msg.range_max else msg.range_max,

            'min_left_25_90': min_left_25_90,
            'min_right_25_90': min_right_25_90
        }

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
        #rospy.logwarn(f"{'  ' * depth}Node at depth {depth}: x={node.x}, y={node.y}, visited={node.visited}")
        
        # Rufe die Funktion rekursiv für alle verbundenen Nodes auf
        if node.node_1:
            rospy.logwarn(f"{'  ' * depth}node_1: {node.node_1.x}, {node.node_1.y}, {node.node_1.number_of_roads}")
            self.log_all_nodes(node.node_1, depth + 1)
        if node.node_2:
            rospy.logwarn(f"{'  ' * depth}node_2: {node.node_2.x}, {node.node_2.y}, {node.node_2.number_of_roads}")
            self.log_all_nodes(node.node_2, depth + 1)
        if node.node_3:
            rospy.logwarn(f"{'  ' * depth}node_3: {node.node_3.x}, {node.node_3.y}, {node.node_3.number_of_roads}")
            self.log_all_nodes(node.node_3, depth + 1)

    def just_once_called(self):
        self.called = True
        rospy.logwarn(f"Im here")
        cmd_vel = Twist()
        start_time= rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < 2.5:
            cmd_vel.angular.z=0.1
            self.cmd_vel_pub.publish(cmd_vel)
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < 2.5:
            cmd_vel.angular.z=-0.1
            self.cmd_vel_pub.publish(cmd_vel)
        cmd_vel.angular.z=0
        self.cmd_vel_pub.publish(cmd_vel)
    
    def move(self):
        cmd_vel = Twist()
        front_dist = self.distances.get('front', float('inf'))
        right_back = self.distances.get('right_back', float('inf'))
        left_back = self.distances.get('left_back', float('inf'))
        left = self.distances.get('left',float('inf'))
        right = self.distances.get('right',float('inf'))

        if not self.called:
            self.just_once_called()
        
        #self.log_all_nodes(self.curr_parent)
        #rospy.loginfo(f"current Node : {self.curr_node.x}, {self.curr_node.y}")
        if  (1000 > left_back > 0.6 and left > 0.6 ) or (1000 > right_back > 0.6 and right > 0.6): #because at the beginnig i receive inf from the laser

            if self.curr_node.parent is None or (not (self.current_x_real - 0.61 < self.curr_node.parent.x < self.current_x_real + 0.61) or
                                                    not (self.current_y_real - 0.61 < self.curr_node.parent.y < self.current_y_real + 0.61)
                                                ):
                rospy.loginfo(f"x: {self.curr_node.x}, y: {self.curr_node.y}")
                if self.curr_node.x == 0 and self.curr_node.y == 0:
                    self.curr_node.x = self.current_x_real
                    self.curr_node.y = self.current_y_real
                    self.curr_node.number_of_roads = self.number_of_roads()
                    #rospy.logwarn(f"new_x: {self.curr_node.x}, new_y: {self.curr_node.x}, roads {self.curr_node.number_of_roads}")
                
                #rospy.loginfo(f"parent_x: {self.curr_node.parent.x}, y: {self.curr_node.parent.y}")
                direction= self.get_the_new_orientation() #after this line the current node is changed to the child if the child is not already existing
                rospy.loginfo(f"direction {direction}")

                if direction == 1 or direction == -1:
                    cmd_vel.linear.x = 0.0
                    self.cmd_vel_pub.publish(cmd_vel)
                    self.rotate(direction)
                
                elif direction == 0:
                    if front_dist < 0.3:
                        cmd_vel.linear.x = 0.1
                    else:
                        cmd_vel.linear.x = 0.2
                    self.cmd_vel_pub.publish(cmd_vel)
                    rospy.sleep(1)
                
                else: # == -2
                    # No movement if no suitable direction found
                    self.return_to_the_last_crossroads()
                    self.curr_node = self.curr_node.parent

            if hasattr(self, 'last_direction') and front_dist > 0.2:
                self.move_forward()
                    
        else:
            # Only move forward if front path is clear and we have a valid direction
            if hasattr(self, 'last_direction') and front_dist > 0.2:
                self.move_forward()

        if self.laser_scan_msg != 0:
                if self.check_closed_way():
                    rospy.logwarn(f"closed way")
                    self.return_to_the_last_crossroads()
                    self.curr_node = self.curr_node.parent #after returning to the parent set the current node to the parent

    def move_forward(self):
        left_dist = self.distances.get('left', float('inf'))
        right_dist = self.distances.get('right', float('inf'))
        front_dist = self.distances.get('front', float('inf'))
        min_left_25_90  = self.distances.get('min_left_25_90', float('inf'))
        min_right_25_90 = self.distances.get('min_right_25_90', float('inf'))

        cmd_vel = Twist()
        cmd_vel.linear.x = 0.2
        if left_dist > 0.6 and right_dist > 0.6 :
            cmd_vel.angular.z = 0
        #laufe neben der linken Wand
        elif self.move_beside_left:
            if min_right_25_90 < 0.2:
                cmd_vel.angular.z = 0.2
                cmd_vel.linear.x = 0.15
            elif min_left_25_90 < 0.2:
                cmd_vel.angular.z = -0.2
            elif min_left_25_90 < 0.25:
                cmd_vel.angular.z = -0.1
            elif min_left_25_90 < 0.35:
                cmd_vel.angular.z = 0
                cmd_vel.linear.x = 0.23
            elif min_left_25_90 > 0.32:
                cmd_vel.angular.z = 0.1
                cmd_vel.linear.x = 0.23

        else: #laufe neben der rechten Wand
            if min_left_25_90 < 0.2:
                cmd_vel.angular.z = -0.2
                cmd_vel.linear.x = 0.15
            elif min_right_25_90 < 0.2:
                cmd_vel.angular.z = 0.2
            elif min_right_25_90 < 0.25:
                cmd_vel.angular.z = 0.1
            elif min_right_25_90 < 0.3:
                cmd_vel.angular.z = 0
                cmd_vel.linear.x = 0.23
            elif min_right_25_90 > 0.32:
                cmd_vel.angular.z = -0.1
                cmd_vel.linear.x = 0.23

        if front_dist < 0.3:
            cmd_vel.linear.x = 0.1
        elif front_dist < 0.2:
            cmd_vel.linear.x = 0
        self.cmd_vel_pub.publish(cmd_vel)

        #rospy.loginfo(f"move: {self.move_beside_left}, left : {min_left_25_90}, right {min_right_25_90}, z: {cmd_vel.angular.z}")

    def rotate(self, direction):
        cmd_vel = Twist()
        #rospy.loginfo(f"yaw {self.current_yaw}")
        if 0.75 < abs(self.current_yaw) < 2.25: #he is looking right or left
            self.to_direction_up_down= True
            if self.current_yaw < 0:
                rotate_x = self.current_x_real - (direction * 0.5)
            else:
                rotate_x = self.current_x_real + (direction * 0.5)
            rotate_y = self.current_y_real
            #rospy.loginfo(f"rotate_y : {rotate_y}, rotate_x : {rotate_x}")
            angle_diff = self.get_angle_to_goal(rotate_x,rotate_y)

            while abs(angle_diff) > 0.05:
                if abs(angle_diff) > 0.4:
                    cmd_vel.angular.z = 0.6 if angle_diff > 0 else -0.6
                elif abs(angle_diff) > 0.2:
                    cmd_vel.angular.z = 0.3 if angle_diff > 0 else -0.3
                elif abs(angle_diff) > 0.1:
                    cmd_vel.angular.z = 0.2 if angle_diff > 0 else -0.2
                elif abs(angle_diff) > 0.05:
                    cmd_vel.angular.z = 0.05 if angle_diff > 0 else -0.05
                self.cmd_vel_pub.publish(cmd_vel)
                angle_diff = self.get_angle_to_goal(rotate_x,rotate_y)
        else:
            self.to_direction_up_down= False
            rotate_x = self.current_x_real
            if self.current_yaw < -2.25 or self.current_yaw > 2.25:
                rotate_y = self.current_y_real + (direction * 0.5)
            else:
                rotate_y = self.current_y_real - (direction * 0.5)
            #rospy.loginfo(f"rotate_y : {rotate_y}, rotate_x : {rotate_x}")
            angle_diff = self.get_angle_to_goal(rotate_x,rotate_y)

            while abs(angle_diff) > 0.05:
                if abs(angle_diff) > 0.4:
                    cmd_vel.angular.z = 0.6 if angle_diff > 0 else -0.6
                elif abs(angle_diff) > 0.2:
                    cmd_vel.angular.z = 0.3 if angle_diff > 0 else -0.3
                elif abs(angle_diff) > 0.1:
                    cmd_vel.angular.z = 0.2 if angle_diff > 0 else -0.2
                elif abs(angle_diff) > 0.05:
                    cmd_vel.angular.z = 0.05 if angle_diff > 0 else -0.05
                self.cmd_vel_pub.publish(cmd_vel)
                angle_diff = self.get_angle_to_goal(rotate_x,rotate_y)

        # Stop rotation
        cmd_vel.angular.z = 0
        self.cmd_vel_pub.publish(cmd_vel)
        rospy.sleep(1)

        left_dist = self.distances.get('left', float('inf'))
        right_dist = self.distances.get('right', float('inf'))
        if (left_dist < right_dist):
            self.move_beside_left = True
        else:
            self.move_beside_left = False

    def number_of_roads(self): #ToDo : integrate it with check closed way
        front_dist = self.distances.get('front', float('inf'))
        left_dist = self.distances.get('left', float('inf'))
        right_dist = self.distances.get('right', float('inf'))

        number_of_roads =0
        if front_dist > 0.65:
            number_of_roads += 1
        if left_dist > 0.65:
            number_of_roads +=1
        if right_dist > 0.65:
            number_of_roads +=1
        return number_of_roads

          
    def check_if_node_saved(self):
        if self.curr_node.node_1 is None:
            self.curr_node.node_1 = Node() #initialize the node 
            self.curr_node.node_1.visited = True #set it to visited
            self.curr_node.node_1.parent = self.curr_node #add the node itself as parent to move to the child
            self.curr_node = self.curr_node.node_1 #set the current nod to the first son, to add the x and y when you arrive it
            #rospy.logwarn(f"moved to the first")
            return False
        elif self.curr_node.node_2 is None and self.curr_node.number_of_roads >= 2:
            self.curr_node.node_2 = Node() 
            self.curr_node.node_2.visited = True
            self.curr_node.node_2.parent = self.curr_node
            self.curr_node = self.curr_node.node_2
            #rospy.logwarn(f"moved to the second")
            return False
        elif self.curr_node.node_3 is None and self.curr_node.number_of_roads == 3:
            self.curr_node.node_3 = Node()
            self.curr_node.node_3.visited = True
            self.curr_node.node_3.parent = self.curr_node
            self.curr_node = self.curr_node.node_3
            #rospy.logwarn(f"moved to the third")
            return False
        else:
            return True


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
        front_dist = self.distances.get('front', float('inf'))

        rospy.loginfo(f"right: {right_dist}")
        rospy.loginfo(f"left: {left_dist}")
        rospy.loginfo(f"front: {front_dist}")

        # Check if paths are clear
        left_clear = left_dist > 0.66
        right_clear = right_dist > 0.66
        front_dist = front_dist > 0.66
        
        if self.check_if_node_saved():
            return -2
        
        if right_clear:
            self.last_direction = -1 
            return -1
        elif front_dist:
            self.last_direction = 0 
            return 0
        elif left_clear:
            self.last_direction = 1 
            return 1


    def get_new_goal(self,direction, distance):
        if 0.75 < abs(self.current_yaw) < 2.25:
            if direction == 1 or direction == -1: #y is fixed
                y= self.current_y_real
                x= self.get_the_x(distance)
            else: #x is fixed
                x= self.current_x_real
                y= self.get_the_y(distance)
        else:
            if direction == 1 or direction == -1:  #x is fixed
                x= self.current_x_real
                y= self.get_the_y(distance)
                rospy.loginfo(f"goal x : {x}, curr x : {self.current_x_real}, goal y : {y}")
            else: #y is fixed
                y= self.current_y_real
                x= self.get_the_x(distance)
                      
        self.temp_goal_x = x
        self.temp_goal_y = y
            

    def get_the_y(self,distance):
        if self.current_y_real < 0:
            return -(distance + abs(self.current_y_real))
        else:
            return distance + abs(self.current_y_real)

    def get_the_x(self,distance):
        if self.current_x_real < 0:
            return -(distance + abs(self.current_x_real))
        else:
            return distance + abs(self.current_x_real)

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

        rospy.loginfo(f"in the return to the parent: {goal_x},   {goal_y}")
        distance_to_goal = self.get_distance_to_temp_goal(goal_x, goal_y)
        cmd_vel = Twist()
        while distance_to_goal > 0.1:
            self.move_to_goal(goal_x, goal_y)
            distance_to_goal = self.get_distance_to_temp_goal(goal_x, goal_y)

        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)
        rospy.loginfo(f"returned")


    '''
    return true if the road closed, false if not
    it has to check it using the laser data (if there is no a x_distance between the points of the laser then it is closed)
    the x_distance is smaller a little bit than the spaces between the walls (it needs to be tested and adjasted rightly)
    '''
    def check_closed_way(self):
        if not self.laser_scan_msg:
            return False

        MIN_PASSAGE_WIDTH = 0.58  # Minimum width needed for robot to pass through
        MAX_RANGE_CHECK = 3.0    # Maximum distance to check for obstacles
        
        ANGLE_RANGES = {
            'front': [(320, 360), (0, 40)],
            'left': [(35, 110)],
            'right': [(250, 325)]
        }

        angle_min = self.laser_scan_msg.angle_min
        angle_increment = self.laser_scan_msg.angle_increment
        ranges = self.laser_scan_msg.ranges

        def analyze_direction(angle_range):
            start_deg, end_deg = angle_range
        
            # Convert to radians, handling 360-degree format
            start_rad = math.radians(start_deg % 360)
            end_rad = math.radians(end_deg % 360)
            
            # Calculate indices based on the converted radians
            start_idx = int((start_rad - angle_min) / angle_increment)
            end_idx = int((end_rad - angle_min) / angle_increment)
            
            # Handle wraparound for 360 degrees
            if end_idx < start_idx:
                end_idx += len(ranges)
                
            # Ensure indices are within bounds
            start_idx = max(0, min(start_idx, len(ranges) - 1))
            end_idx = max(0, min(end_idx, len(ranges) - 1))
            
            rospy.loginfo(f"Analyzing range {start_deg}° to {end_deg}°")
            rospy.loginfo(f"Indices: {start_idx} to {end_idx}")
            
            valid_readings = []
            for i in range(start_idx, end_idx):
                # Handle wraparound for indices
                idx = i % len(ranges)
                range_val = ranges[idx]
                if self.laser_scan_msg.range_min <= range_val <= MAX_RANGE_CHECK:
                    valid_readings.append(range_val)
            
            if not valid_readings:
                rospy.loginfo(f"No valid readings in range {start_deg}°-{end_deg}°")
                return True
                
            max_gap = 0
            for i in range(len(valid_readings) - 1):
                gap = abs(valid_readings[i] - valid_readings[i + 1])
                max_gap = max(max_gap, gap)
            
            rospy.loginfo(f"Max gap in range {start_deg}°-{end_deg}°: {max_gap:.3f}m")
            return max_gap < MIN_PASSAGE_WIDTH

        # Check each direction
        for direction, angle_ranges in ANGLE_RANGES.items():
            for angle_range in angle_ranges:
                if not analyze_direction(angle_range):
                    rospy.loginfo(f"Passage found in {direction} direction")
                    return False
                
        rospy.loginfo("All directions appear closed")
        return True
    
    '''
    def check_closed_way(self):
        left_dist = self.distances.get('left', float('inf'))
        right_dist = self.distances.get('right', float('inf'))
        front_dist = self.distances.get('front', float('inf'))
        cmd_vel = Twist()
        #rospy.loginfo(f"right: {right_dist}, left {left_dist}, front: {front_dist}")
        if left_dist < 0.6 and right_dist < 0.6 and front_dist < 0.3:
            cmd_vel.linear.x = 0
            self.cmd_vel_pub.publish(cmd_vel)
            return True
        return False
    '''


    def move_to_goal(self,goal_x,goal_y):

        # Calculate distance and angle to goal
        distance_to_goal = self.get_distance_to_temp_goal(goal_x, goal_y)
        #rospy.loginfo(f"x: {goal_x} , y: {goal_y}")
        angle_diff = self.get_angle_to_goal(goal_x, goal_y)
        #rospy.loginfo(f"angle {distance_to_goal}")
        cmd_vel = Twist()
        
        if abs(angle_diff) > 0.4:
            cmd_vel.angular.z = 0.6 if angle_diff > 0 else -0.6
            cmd_vel.linear.x = 0.0
        elif abs(angle_diff) > 0.2:
            cmd_vel.angular.z = 0.3 if angle_diff > 0 else -0.3
            if distance_to_goal > 0.5:
                cmd_vel.linear.x = 0.4
            elif distance_to_goal > 0.3:
                cmd_vel.linear.x = 0.2
            elif distance_to_goal > 0.1:
                cmd_vel.linear.x = 0.15
        elif abs(angle_diff) > 0.1:
            cmd_vel.angular.z = 0.2 if angle_diff > 0 else -0.2
            if distance_to_goal > 1:
                cmd_vel.linear.x = 0.5
            elif distance_to_goal > 0.5:
                cmd_vel.linear.x = 0.4
            elif distance_to_goal > 0.3:
                cmd_vel.linear.x = 0.2
            elif distance_to_goal > 0.1:
                cmd_vel.linear.x = 0.15
        else:
            if distance_to_goal > 1:
                cmd_vel.linear.x = 0.5
            elif distance_to_goal > 0.5:
                cmd_vel.linear.x = 0.4
            elif distance_to_goal > 0.3:
                cmd_vel.linear.x = 0.2
            elif distance_to_goal > 0.1:
                cmd_vel.linear.x = 0.15
        '''
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
        '''
        self.cmd_vel_pub.publish(cmd_vel)
        
    
    def control_loop(self):
        # Hauptkontrollschleife
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.get_distance_to_temp_goal(self.goal_x,self.goal_y) < 0.1:
                rospy.loginfo("goal reached")
                self.stop_robot()
                break
            else :
                self.move()
                rate.sleep()

    def stop_robot(self):
        """Generate velocity command to stop the robot."""
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)

if __name__ == '__main__':
    try:
        explorer = Turtlebot3Explorer()
        explorer.control_loop()
        #rospy.spin()  #to keep the node running
    except rospy.ROSInterruptException as e:
        rospy.loginfo(e)