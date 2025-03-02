#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist, PointStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from tf.transformations import euler_from_quaternion
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


class Turtlebot3Explorer:
    def __init__(self):
        rospy.init_node('aufgabe3', anonymous=True)

        # SLAM und Kartierungsvariablen
        self.return_to_x = []  # Store X coordinates of blocked dead-ends
        self.return_to_y = []  # Store Y coordinates of blocked dead-ends

        self.current_x_real = 0.03374290466308594
        self.current_y_real = -0.0013123006792739
        self.goal_x = None
        self.goal_y = None

        self.current_yaw = 0.0
        
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
        self.direction = 0

        #wurzel erstellen
        if self.curr_node is None:
            node = Node()
            node.x = self.current_x_real
            node.y = self.current_y_real
            node.visited = True
            self.curr_node = node
            #self.curr_node.parent = node #the root is the parent of itself
            self.curr_parent = node
            self.curr_node.number_of_roads = 3

        self.move_beside_left = True
        self.distances = {}  # Store distances to walls in different directions
        self.distances_min = {}

        # Publisher und Subscriber
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        self.scan_sub = rospy.Subscriber("scan", LaserScan, self.laser_scan_callback)
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)
        self.map_sub = rospy.Subscriber("map", OccupancyGrid, self.map_callback)
        self.temp_goal_pub = rospy.Publisher('/temp_goal_pub', Marker, queue_size=10)
        self.goal_sub = rospy.Subscriber('/maze_goal', PointStamped, self.set_goal)


    def set_goal(self, msg: Odometry):
        self.goal_x = msg.point.x
        self.goal_y = msg.point.y

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
        
        if node.node_1:
            self.log_all_nodes(node.node_1, depth + 1)
        if node.node_2:
            self.log_all_nodes(node.node_2, depth + 1)
        if node.node_3:
            self.log_all_nodes(node.node_3, depth + 1)

    def just_once_called(self):
        self.called = True
        cmd_vel = Twist()
        start_time= rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < 1:
            cmd_vel.angular.z=0.1
            self.cmd_vel_pub.publish(cmd_vel)
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < 1:
            cmd_vel.angular.z=-0.1
            self.cmd_vel_pub.publish(cmd_vel)
        cmd_vel.angular.z=0
        self.cmd_vel_pub.publish(cmd_vel)
    
    def move(self):
        cmd_vel = Twist()
        front_dist = self.distances.get('front', float('inf'))
        direct = self.distances.get('direct', float('inf'))
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
                if self.curr_node.x == 0 and self.curr_node.y == 0:
                    self.curr_node.x = self.current_x_real
                    self.curr_node.y = self.current_y_real
                    self.curr_node.number_of_roads = self.number_of_roads()
                
                self.direction= self.get_the_new_orientation() #after this line the current node is changed to the child if the child is not already existing

                if self.direction == 1 or self.direction == -1:
                    cmd_vel.linear.x = 0.0
                    self.cmd_vel_pub.publish(cmd_vel)
                    self.rotate(self.direction)
                
                elif self.direction == 0:
                    if self.get_distance_to_temp_goal(self.temp_goal_x,self.temp_goal_y) < 0.2:
                        if right > 0.5 and left > 0.5:
                            self.get_new_goal(direct-0.3,0.3,True) # hier ist true oder false egal weil diff wird 0 sein
                        elif (left < right):
                            self.get_new_goal(direct-0.3,left,False) # True for right and False for left
                        else:
                            self.get_new_goal(direct-0.3,right,True)

                    if front_dist < 0.3:
                        cmd_vel.linear.x = 0.1
                    else:
                        cmd_vel.linear.x = 0.22
                    self.cmd_vel_pub.publish(cmd_vel)
                
                else: # == -2
                    # No movement if no suitable direction found
                    self.return_to_the_last_crossroads()
                    self.curr_node = self.curr_node.parent

            if hasattr(self, 'last_direction') and front_dist > 0.1:
                if self.get_distance_to_temp_goal(self.temp_goal_x,self.temp_goal_y) < 0.2:
                    if right > 0.5 and left > 0.5:
                        self.get_new_goal(direct-0.3,0.3,True) # hier ist true oder false egal weil diff wird 0 sein
                    elif (left < right):
                        self.get_new_goal(direct-0.3,left,False) # True for right and False for left
                    else:
                        self.get_new_goal(direct-0.3,right,True)
                        
                self.move_to_goal(self.temp_goal_x,self.temp_goal_y)
                
                    
        else:
            # Only move forward if front path is clear and we have a valid direction
            if hasattr(self, 'last_direction') and front_dist > 0.1:
                if self.get_distance_to_temp_goal(self.temp_goal_x,self.temp_goal_y) < 0.2:
                    if right > 0.5 and left > 0.5:
                        self.get_new_goal(direct-0.3,0.3,True) # hier ist true oder false egal weil diff wird 0 sein
                    elif (left < right):
                        self.get_new_goal(direct-0.3,left,False) # True for right and False for left
                    else:
                        self.get_new_goal(direct-0.3,right,True)

                self.move_to_goal(self.temp_goal_x,self.temp_goal_y)
                

        if self.laser_scan_msg != 0:
                if self.check_closed_way():
                    self.return_to_the_last_crossroads()
                    self.curr_node = self.curr_node.parent #after returning to the parent set the current node to the parent


    def rotate(self, direction):
        cmd_vel = Twist()

        if 0.75 < abs(self.current_yaw) < 2.25: #he is looking right or left
            self.to_direction_up_down= True
            if self.current_yaw < 0:
                rotate_x = self.current_x_real - (direction * 0.5)
            else:
                rotate_x = self.current_x_real + (direction * 0.5)
            rotate_y = self.current_y_real
            angle_diff = self.get_angle_to_goal(rotate_x,rotate_y)

            while abs(angle_diff) > 0.05:
                if abs(angle_diff) > 0.6:
                    cmd_vel.angular.z = 2 if angle_diff > 0 else -2
                elif abs(angle_diff) > 0.5:
                    cmd_vel.angular.z = 1.5 if angle_diff > 0 else -1.5
                elif abs(angle_diff) > 0.4:
                    cmd_vel.angular.z = 1 if angle_diff > 0 else -1
                elif abs(angle_diff) > 0.3:
                    cmd_vel.angular.z = 0.8 if angle_diff > 0 else -0.8
                elif abs(angle_diff) > 0.2:
                    cmd_vel.angular.z = 0.2 if angle_diff > 0 else -0.2
                elif abs(angle_diff) > 0.1:
                    cmd_vel.angular.z = 0.1 if angle_diff > 0 else -0.1
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
            angle_diff = self.get_angle_to_goal(rotate_x,rotate_y)

            while abs(angle_diff) > 0.05:
                if abs(angle_diff) > 0.6:
                    cmd_vel.angular.z = 2 if angle_diff > 0 else -2
                elif abs(angle_diff) > 0.5:
                    cmd_vel.angular.z = 1.5 if angle_diff > 0 else -1.5
                elif abs(angle_diff) > 0.4:
                    cmd_vel.angular.z = 1 if angle_diff > 0 else -1
                elif abs(angle_diff) > 0.3:
                    cmd_vel.angular.z = 0.8 if angle_diff > 0 else -0.8
                elif abs(angle_diff) > 0.2:
                    cmd_vel.angular.z = 0.2 if angle_diff > 0 else -0.2
                elif abs(angle_diff) > 0.1:
                    cmd_vel.angular.z = 0.1 if angle_diff > 0 else -0.1
                elif abs(angle_diff) > 0.05:
                    cmd_vel.angular.z = 0.05 if angle_diff > 0 else -0.05

                self.cmd_vel_pub.publish(cmd_vel)
                angle_diff = self.get_angle_to_goal(rotate_x,rotate_y)

        # Stop rotation
        cmd_vel.angular.z = 0
        self.cmd_vel_pub.publish(cmd_vel)
        
        left_dist = self.distances.get('left', float('inf'))
        right_dist = self.distances.get('right', float('inf'))
        direct = self.distances.get('direct', float('inf'))

        if right_dist > 0.5 and left_dist > 0.5:
            self.get_new_goal(direct-0.3,0.3,True) # hier ist true oder false egal weil diff wird 0 sein
        elif (left_dist < right_dist):
            self.get_new_goal(direct-0.3,left_dist,False) # True for right and False for left
        else:
            self.get_new_goal(direct-0.3,right_dist,True)

    def number_of_roads(self): 
        direct = self.distances.get('direct', float('inf'))
        left_dist = self.distances.get('left', float('inf'))
        right_dist = self.distances.get('right', float('inf'))


        number_of_roads =0
        if direct > 0.6:
            number_of_roads += 1
        if left_dist > 0.6:
            number_of_roads +=1
        if right_dist > 0.6:
            number_of_roads +=1
        return number_of_roads

    def check_if_node_saved(self):
        if self.curr_node.node_1 is None:
            self.curr_node.node_1 = Node() #initialize the node 
            self.curr_node.node_1.visited = True #set it to visited
            self.curr_node.node_1.parent = self.curr_node #add the node itself as parent to move to the child
            self.curr_node = self.curr_node.node_1 #set the current nod to the first son, to add the x and y when you arrive it
            return False
        elif self.curr_node.node_2 is None and self.curr_node.number_of_roads >= 2:
            self.curr_node.node_2 = Node() 
            self.curr_node.node_2.visited = True
            self.curr_node.node_2.parent = self.curr_node
            self.curr_node = self.curr_node.node_2
            return False
        elif self.curr_node.node_3 is None and self.curr_node.number_of_roads == 3:
            self.curr_node.node_3 = Node()
            self.curr_node.node_3.visited = True
            self.curr_node.node_3.parent = self.curr_node
            self.curr_node = self.curr_node.node_3
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
        if 0.75 < abs(self.current_yaw) < 2.25:
            #x is fixed
                if (right_wall and 0.75 < self.current_yaw < 2.25) or (not right_wall and -2.25 < self.current_yaw < -0.75): # (rechte Wand und guckt rechts) or (linke Wand und guckt links)
                    x = self.current_x_real - difference
                elif (not right_wall and 0.75 < self.current_yaw < 2.25) or (right_wall and -2.25 < self.current_yaw < -0.75): # (linke Wand und guckt rechts) or (rechte Wand und guckt links)
                    x= self.current_x_real + difference

                y= self.get_the_y(distance)
        else:
            #y is fixed
                if (right_wall and abs(self.current_yaw) > 2.25) or (not right_wall and abs(self.current_yaw) < 0.75): # (rechte Wand und guckt oben) or (linke Wand und guckt unten)
                    y= self.current_y_real - difference
                elif (not right_wall and abs(self.current_yaw) > 2.25) or (right_wall and abs(self.current_yaw) < 0.75): # (linke Wand und guckt oben) or (rechte Wand und guckt unten)
                    y= self.current_y_real + difference

                x= self.get_the_x(distance)
                      
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
            return
        goal_x, goal_y = self.curr_node.parent.x, self.curr_node.parent.y

        distance_to_goal = self.get_distance_to_temp_goal(goal_x, goal_y)

        cmd_vel = Twist()

        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)
        while distance_to_goal > 0.1:
            self.move_to_goal(goal_x, goal_y)
            distance_to_goal = self.get_distance_to_temp_goal(goal_x, goal_y)

        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)


    '''
    return true if the road closed, false if not
    it has to check it using the laser data (if there is no a x_distance between the points of the laser then it is closed)
    the x_distance is smaller a little bit than the spaces between the walls (it needs to be tested and adjasted rightly)
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

        if abs(angle_diff) > 0.6:
            cmd_vel.angular.z = 2.84 if angle_diff > 0 else -2.84
            cmd_vel.linear.x = 0.0
        if abs(angle_diff) > 0.5:
            cmd_vel.angular.z = 2 if angle_diff > 0 else -2
            cmd_vel.linear.x = 0.0
        elif abs(angle_diff) > 0.4:
            cmd_vel.angular.z = 1.5 if angle_diff > 0 else -1.5
            cmd_vel.linear.x = 0.0
        elif abs(angle_diff) > 0.3:
            cmd_vel.angular.z = 1 if angle_diff > 0 else -1
            cmd_vel.linear.x = 0.0
        else:
            if distance_to_goal > 0.22:
                cmd_vel.linear.x = 0.22
                if abs(angle_diff) > 0.2:
                    cmd_vel.angular.z = 0.15 if angle_diff > 0 else -0.15
                elif abs(angle_diff) > 0.1:
                    cmd_vel.angular.z = 0.1 if angle_diff > 0 else -0.1
                elif abs(angle_diff) > 0.05:
                    cmd_vel.angular.z = 0.05 if angle_diff > 0 else -0.05
                else:
                    cmd_vel.angular.z = 0
            elif distance_to_goal > 0.2:
                cmd_vel.linear.x = 0.21
                if abs(angle_diff) > 0.2:
                    cmd_vel.angular.z = 0.15 if angle_diff > 0 else -0.15
                elif abs(angle_diff) > 0.1:
                    cmd_vel.angular.z = 0.1 if angle_diff > 0 else -0.1
                elif abs(angle_diff) > 0.05:
                    cmd_vel.angular.z = 0.05 if angle_diff > 0 else -0.05
                else:
                    cmd_vel.angular.z = 0
            elif distance_to_goal > 0.1:
                cmd_vel.linear.x = 0.15
                if abs(angle_diff) > 0.2:
                    cmd_vel.angular.z = 0.15 if angle_diff > 0 else -0.15
                elif abs(angle_diff) > 0.1:
                    cmd_vel.angular.z = 0.1 if angle_diff > 0 else -0.1
                elif abs(angle_diff) > 0.05:
                    cmd_vel.angular.z = 0.05 if angle_diff > 0 else -0.05
                else:
                    cmd_vel.angular.z = 0
            else:
                cmd_vel.linear.x = 0

        self.cmd_vel_pub.publish(cmd_vel)
    

    def control_loop(self):
        # Hauptkontrollschleife
        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            if self.goal_x == 0: # can not write self.goal_x because he is considering the 0 as None
                if not self.called:
                    self.just_once_called() 
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