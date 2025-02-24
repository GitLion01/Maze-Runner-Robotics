#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist, Point, PoseArray
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from tf.transformations import euler_from_quaternion
from collections import deque
from turtlebot_maze_navigation.msg import MapData
from visualization_msgs.msg import Marker

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
        self.already_oriented = False

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
            self.curr_node.number_of_roads = 3 #need to make this dinamic

        self.move_beside_left = True
        self.distances = {}  # Store distances to walls in different directions
        self.distances_min = {}

        # Publisher und Subscriber
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        self.scan_sub = rospy.Subscriber("scan", LaserScan, self.laser_scan_callback)
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)
        self.map_sub = rospy.Subscriber("map", OccupancyGrid, self.map_callback)
        self.temp_goal_pub = rospy.Publisher('/temp_goal_pub', Marker, queue_size=10)

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
        direct_idx = int((0 - msg.angle_min) / angle_increment)
        left_idx = int((math.radians(90) - msg.angle_min) / angle_increment)
        left_back_idx = int((math.radians(103) - msg.angle_min) / angle_increment)
        #left_front_idx = int((math.radians(85) - msg.angle_min) / angle_increment)

        right_idx = int((math.radians(-90) - msg.angle_min) / angle_increment)
        right_back_idx = int((math.radians(-103) - msg.angle_min) / angle_increment)
        #right_front_idx = int((math.radians(-85) - msg.angle_min) / angle_increment)


        # Calculate indices for 25 and 90 degrees
        left_25_idx = int((math.radians(25) - msg.angle_min) / angle_increment)
        left_90_idx = int((math.radians(90) - msg.angle_min) / angle_increment)
        
        # Get minimum distance in the 25-90 degree range
        left_25_90_range = msg.ranges[left_25_idx:left_90_idx + 1]
        # Filter out invalid readings
        valid_left_readings = [r for r in left_25_90_range if msg.range_min <= r <= msg.range_max]
        min_left_25_90 = min(valid_left_readings) if valid_left_readings else msg.range_max

        # Calculate indices for 25 and 90 degrees
        right_25_idx = int((math.radians(-25) - msg.angle_min) / angle_increment)
        right_90_idx = int((math.radians(-90) - msg.angle_min) / angle_increment)
        
        # Get minimum distance in the 25-90 degree range
        right_25_90_range = msg.ranges[right_90_idx:right_25_idx + 1]
        # Filter out invalid readings
        valid_right_readings = [r for r in right_25_90_range if msg.range_min <= r <= msg.range_max]
        min_right_25_90 = min(valid_right_readings) if valid_right_readings else msg.range_max

        # Get indices for both segments
        idx_340_to_360 = [i for i, _ in enumerate(msg.ranges) if math.radians(340) <= msg.angle_min + i * msg.angle_increment <= 2*math.pi]
        idx_0_to_20 = [i for i, _ in enumerate(msg.ranges) if 0 <= msg.angle_min + i * msg.angle_increment <= math.radians(20)]
        front_indices = idx_340_to_360 + idx_0_to_20
    
        front_range = [msg.ranges[i] for i in front_indices]
        valid_front_readings = [r for r in front_range if msg.range_min <= r <= msg.range_max]
        min_front = min(valid_front_readings) if valid_front_readings else msg.range_max

        # Get readings
        self.distances = {
            'direct': msg.ranges[direct_idx] if msg.range_min <= msg.ranges[direct_idx] <= msg.range_max else msg.range_max,
            'front' : min_front,
            'left': msg.ranges[left_idx] if msg.range_min <= msg.ranges[left_idx] <= msg.range_max else msg.range_max,
            'left_back': msg.ranges[left_back_idx] if msg.range_min <= msg.ranges[left_back_idx] <= msg.range_max else msg.range_max,
            #'left_front': msg.ranges[left_front_idx] if msg.range_min <= msg.ranges[left_front_idx] <= msg.range_max else msg.range_max,

            'right': msg.ranges[right_idx] if msg.range_min <= msg.ranges[right_idx] <= msg.range_max else msg.range_max,
            'right_back': msg.ranges[right_back_idx] if msg.range_min <= msg.ranges[right_back_idx] <= msg.range_max else msg.range_max,
            #'right_front': msg.ranges[right_front_idx] if msg.range_min <= msg.ranges[right_front_idx] <= msg.range_max else msg.range_max,

            'min_left_25_90': min_left_25_90,
            'min_right_25_90': min_right_25_90,

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
        
        #self.log_all_nodes(self.curr_parent)
        if  (left > 0.58 ) or (right > 0.58): #because at the beginnig i receive inf from the laser
            rospy.sleep(0.5) # to make the robot move a little bit forward
            if self.curr_node.parent is None or (not (self.current_x_real - 0.6 < self.curr_node.parent.x < self.current_x_real + 0.6) or
                                                    not (self.current_y_real - 0.6 < self.curr_node.parent.y < self.current_y_real + 0.6)
                                                ):
                rospy.loginfo(f"x: {self.curr_node.x}, y: {self.curr_node.y}")
                if self.curr_node.x == 0 and self.curr_node.y == 0:
                    self.curr_node.x = self.current_x_real
                    self.curr_node.y = self.current_y_real
                    self.curr_node.number_of_roads = self.number_of_roads()
                    rospy.logwarn(f"new_x: {self.curr_node.x}, new_y: {self.curr_node.x}, roads {self.curr_node.number_of_roads}")
                
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
                
                else: # == -2
                    # No movement if no suitable direction found
                    self.return_to_the_last_crossroads()
                    self.curr_node = self.curr_node.parent
                    rospy.loginfo(f"left_back: {left_back}, left: {left}")
                    rospy.loginfo(f"right_back: {right_back}, right: {right}")

            if hasattr(self, 'last_direction') and front_dist > 0.2:
                #self.move_forward()
                self.move_to_goal(self.temp_goal_x,self.temp_goal_y)
                
                    
        else:
            # Only move forward if front path is clear and we have a valid direction
            if hasattr(self, 'last_direction') and front_dist > 0.2:
                #self.move_forward()
                self.move_to_goal(self.temp_goal_x,self.temp_goal_y)
                

        if self.laser_scan_msg != 0:
                if self.check_closed_way():
                    rospy.logwarn(f"closed way")
                    self.return_to_the_last_crossroads()
                    rospy.loginfo(f"2 left_back: {left_back}, left: {left}")
                    rospy.loginfo(f"2 right_back: {right_back}, right: {right}")
                    self.curr_node = self.curr_node.parent #after returning to the parent set the current node to the parent


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

            while abs(angle_diff) > 0.1:
                if abs(angle_diff) > 0.4:
                    cmd_vel.angular.z = 0.6 if angle_diff > 0 else -0.6
                elif abs(angle_diff) > 0.2:
                    cmd_vel.angular.z = 0.3 if angle_diff > 0 else -0.3
                elif abs(angle_diff) > 0.1:
                    cmd_vel.angular.z = 0.2 if angle_diff > 0 else -0.2
                elif abs(angle_diff) > 0.1:
                    cmd_vel.angular.z = 0.1 if angle_diff > 0 else -0.1
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

            while abs(angle_diff) > 0.1:
                if abs(angle_diff) > 0.4:
                    cmd_vel.angular.z = 0.6 if angle_diff > 0 else -0.6
                elif abs(angle_diff) > 0.2:
                    cmd_vel.angular.z = 0.3 if angle_diff > 0 else -0.3
                elif abs(angle_diff) > 0.1:
                    cmd_vel.angular.z = 0.2 if angle_diff > 0 else -0.2
                elif abs(angle_diff) > 0.1:
                    cmd_vel.angular.z = 0.1 if angle_diff > 0 else -0.1
                self.cmd_vel_pub.publish(cmd_vel)
                angle_diff = self.get_angle_to_goal(rotate_x,rotate_y)

        # Stop rotation
        cmd_vel.angular.z = 0
        self.cmd_vel_pub.publish(cmd_vel)
        rospy.sleep(0.2)
        
        left_dist = self.distances.get('left', float('inf'))
        right_dist = self.distances.get('right', float('inf'))
        direct = self.distances.get('direct', float('inf'))

        if right_dist > 0.5 and left_dist > 0.5:
            self.get_new_goal(direct-0.3,0.3,True) # hier ist true oder false egal weil diff wird 0 sein
            rospy.loginfo(f"right: {right_dist}, left: {left_dist}",)
        elif (left_dist < right_dist):
            self.get_new_goal(direct-0.3,left_dist,False) # True for right and False for left
            rospy.loginfo(f"left: {left_dist}")
        else:
            self.get_new_goal(direct-0.3,right_dist,True)
            rospy.loginfo(f"right: {right_dist}")

    def number_of_roads(self): 
        direct = self.distances.get('direct', float('inf'))
        left_dist = self.distances.get('left', float('inf'))
        right_dist = self.distances.get('right', float('inf'))


        number_of_roads =0
        if direct > 0.6:
            number_of_roads += 1
            rospy.loginfo(f"front: {direct}")
        if left_dist > 0.6:
            number_of_roads +=1
            rospy.loginfo(f"left: {left_dist}")
        if right_dist > 0.6:
            number_of_roads +=1
            rospy.loginfo(f"right: {right_dist}")
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
        direct = self.distances.get('direct', float('inf'))

        # Check if paths are clear
        left_clear = left_dist > 0.58
        right_clear = right_dist > 0.58
        direct = direct > 0.58
        
        if self.check_if_node_saved():
            return -2
        
        if right_clear:
            self.last_direction = -1 
            self.already_oriented = False
            return -1
        elif direct:
            self.last_direction = 0 
            return 0
        elif left_clear:
            self.last_direction = 1
            self.already_oriented = False
            return 1


    def get_new_goal(self,distance, dist_left_right, right_wall):
        difference = dist_left_right - 0.3
        rospy.loginfo(f"yaw: {self.current_yaw}, right: {right_wall}")
        if 0.75 < abs(self.current_yaw) < 2.25:
            #x is fixed
                if (right_wall and 0.75 < self.current_yaw < 2.25) or (not right_wall and -2.25 < self.current_yaw < -0.75): # (rechte Wand und guckt rechts) or (linke Wand und guckt links)
                    x = self.current_x_real - difference
                    rospy.loginfo(f"1")
                elif (not right_wall and 0.75 < self.current_yaw < 2.25) or (right_wall and -2.25 < self.current_yaw < -0.75): # (linke Wand und guckt rechts) or (rechte Wand und guckt links)
                    x= self.current_x_real + difference
                    rospy.loginfo(f"2")

                y= self.get_the_y(distance)
                rospy.loginfo(f"diff: {difference}")
                rospy.loginfo(f"goal x : {x}, curr x : {self.current_x_real}")
                rospy.loginfo(f"goal y : {y}, curr x : {self.current_y_real}")
        else:
            #y is fixed
                if (right_wall and abs(self.current_yaw) > 2.25) or (not right_wall and abs(self.current_yaw) < 0.75): # (rechte Wand und guckt oben) or (linke Wand und guckt unten)
                    y= self.current_y_real - difference
                    rospy.loginfo("3")
                elif (not right_wall and abs(self.current_yaw) > 2.25) or (right_wall and abs(self.current_yaw) < 0.75): # (linke Wand und guckt oben) or (rechte Wand und guckt unten)
                    y= self.current_y_real + difference
                    rospy.loginfo(f"4")

                x= self.get_the_x(distance)
                rospy.loginfo(f"diff: {difference}")
                rospy.loginfo(f"goal x : {x}, curr x : {self.current_x_real}")
                rospy.loginfo(f"goal y : {y}, curr x : {self.current_y_real}")
                      
        self.temp_goal_x = x
        self.temp_goal_y = y

        marker = Marker()
        marker.header.frame_id = "map"  # Use the appropriate frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "temp_goals"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1  # Size of the sphere
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.r = 1.0  # Red color
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # Fully opaque
        marker.lifetime = rospy.Duration(0)  # 0 = forever
        
        # Publish the marker
        self.temp_goal_pub.publish(marker)
            

    def get_the_y(self,distance):
        if self.current_y_real < 0:
            if 0.75 < self.current_yaw < 2.25:
                return -(distance + abs(self.current_y_real))
            else : # -2.25 < y < -0.75
                return self.current_y_real + distance

            
        else: # wird ne auftreten weil y ist immer negative
            return distance + abs(self.current_y_real)

    def get_the_x(self,distance):
        if self.current_x_real < 0:
            if -0.75 < self.current_yaw < 0.75: #wird ne auftreten !
                return -(distance + abs(self.current_x_real))
            else: #kann auftreten aber selten von -x to +x
                return self.current_x_real + distance
        else:
            if -0.75 < self.current_yaw < 0.75:
                return abs(self.current_x_real) - distance
            else: # 2.25 < x < 3 and -3 < x < -2.25
                return abs(self.current_x_real) + distance


    def return_to_the_last_crossroads(self):
        if self.curr_node is None:
            rospy.logwarn("Node besitzt kein Parent")
            return
        goal_x, goal_y = self.curr_node.parent.x, self.curr_node.parent.y

        rospy.loginfo(f"in the return to the parent: {goal_x},   {goal_y}, {self.curr_node.parent.number_of_roads}")
        distance_to_goal = self.get_distance_to_temp_goal(goal_x, goal_y)

        cmd_vel = Twist()

        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)
        #rospy.loginfo(f"das x: {cmd_vel.linear.x}")
        while distance_to_goal > 0.15:
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
    '''
    def check_closed_way(self):
        if not self.laser_scan_msg:
            return False

        MIN_PASSAGE_WIDTH = 0.4  # Minimum width needed for robot to pass through
        
        ANGLE_RANGES = {
            'front': [(240, 360), (0, 120)]
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
            
            #rospy.loginfo(f"Analyzing range {start_deg}° to {end_deg}°")
            #rospy.loginfo(f"Indices: {start_idx} to {end_idx}")
            
            valid_readings = []
            for i in range(start_idx, end_idx):
                # Handle wraparound for indices
                idx = i % len(ranges)
                range_val = ranges[idx]
                #if self.laser_scan_msg.range_min <= range_val: # <= MAX_RANGE_CHECK:
                valid_readings.append(range_val)
            
            if not valid_readings:
                #rospy.logwarn(f"No valid readings in range {start_deg}°-{end_deg}°")
                return True
                
            max_gap = 0
            for i in range(len(valid_readings) - 1):
                gap = abs(valid_readings[i] - valid_readings[i + 1])
                max_gap = max(max_gap, gap)
            
            #rospy.loginfo(f"Max gap in range {start_deg}°-{end_deg}°: {max_gap:.3f}m")
            return max_gap < MIN_PASSAGE_WIDTH

        # Check each direction
        for direction, angle_ranges in ANGLE_RANGES.items():
            for angle_range in angle_ranges:
                if not analyze_direction(angle_range):
                    #rospy.logwarn(f"Passage found in {direction} direction")
                    return False
                
        rospy.logwarn("All directions appear closed")
        return True

    '''
    def check_closed_way(self):
        left_dist = self.distances.get('left', float('inf'))
        right_dist = self.distances.get('right', float('inf'))
        direct = self.distances.get('direct', float('inf'))

        if left_dist < 0.4 and right_dist < 0.4 and direct < 0.4:
            return True
        return False
    

    def move_to_goal(self,goal_x,goal_y):

        # Calculate distance and angle to goal
        distance_to_goal = self.get_distance_to_temp_goal(goal_x, goal_y)
        angle_diff = self.get_angle_to_goal(goal_x, goal_y)
        cmd_vel = Twist()

        cmd_vel.linear.x = 0.0
        if not self.already_oriented:
            if abs(angle_diff) > 0.4:
                cmd_vel.angular.z = 0.35 if angle_diff > 0 else -0.35
            elif abs(angle_diff) > 0.3:
                cmd_vel.angular.z = 0.25 if angle_diff > 0 else -0.25
            elif abs(angle_diff) > 0.2:
                cmd_vel.angular.z = 0.15 if angle_diff > 0 else -0.15
            elif abs(angle_diff) > 0.1:
                cmd_vel.angular.z = 0.1 if angle_diff > 0 else -0.1
            elif abs(angle_diff) > 0.05:
                cmd_vel.angular.z = 0.05 if angle_diff > 0 else -0.05
            else:
                cmd_vel.angular.z = 0.0
                self.already_oriented = True
        else:
            self.already_oriented = True
            cmd_vel.angular.z = 0
            if distance_to_goal > 0.5:
                cmd_vel.linear.x = 0.22
            elif distance_to_goal > 0.3:
                cmd_vel.linear.x = 0.2
            elif distance_to_goal > 0.15:
                cmd_vel.linear.x = 0.15
            elif distance_to_goal > 0.1:
                cmd_vel.linear.x = 0.1
                self.already_oriented = False
            if abs(angle_diff) > 0.05:
                cmd_vel.angular.z = 0.05 if angle_diff > 0 else -0.05
            elif abs(angle_diff) > 0.1:
                self.already_oriented = True

        self.cmd_vel_pub.publish(cmd_vel)
    

    def return_to_parent(self,goal_x,goal_y):
        # Calculate distance and angle to goal
        distance_to_goal = self.get_distance_to_temp_goal(goal_x, goal_y)
        angle_diff = self.get_angle_to_goal(goal_x, goal_y)
        cmd_vel = Twist()
        
        if abs(angle_diff) > 0.4:
            cmd_vel.angular.z = 0.6 if angle_diff > 0 else -0.6
            cmd_vel.linear.x = 0.0
        elif abs(angle_diff) > 0.3:
            cmd_vel.angular.z = 0.2 if angle_diff > 0 else -0.2
        elif abs(angle_diff) > 0.2:
            cmd_vel.angular.z = 0.15 if angle_diff > 0 else -0.15
            if distance_to_goal > 0.3:
                cmd_vel.linear.x = 0.18
            elif distance_to_goal > 0.1:
                cmd_vel.linear.x = 0.12
        elif abs(angle_diff) > 0.1:
            cmd_vel.angular.z = 0.1 if angle_diff > 0 else -0.1
            if distance_to_goal > 1:
                cmd_vel.linear.x = 0.4
            elif distance_to_goal > 0.5:
                cmd_vel.linear.x = 0.3
            elif distance_to_goal > 0.3:
                cmd_vel.linear.x = 0.18
            elif distance_to_goal > 0.1:
                cmd_vel.linear.x = 0.12
        else:
            if distance_to_goal > 1:
                cmd_vel.linear.x = 0.4
            elif distance_to_goal > 0.5:
                cmd_vel.linear.x = 0.3
            elif distance_to_goal > 0.3:
                cmd_vel.linear.x = 0.18
            elif distance_to_goal > 0.15:
                cmd_vel.linear.x = 0.15

        self.cmd_vel_pub.publish(cmd_vel)
        

    def control_loop(self):
        # Hauptkontrollschleife
        rate = rospy.Rate(10)
        
        self.just_once_called() 
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