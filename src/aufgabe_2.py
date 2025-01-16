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
    def __init__(self, position):
        self.position = position
        self.neighbors = [] 
        self.trap = False # markierte Nodes geben keinen Weg zum Ziel (d.h. man kann ganze "Teilregionen" als Falle markieren)
        
    def __eq__(self, other):
        return self.position == other.position

    def __hash__(self):
        return hash(self.position)
    """
    def get_neighbors(self, threshold=5):
        x, y = self.position
        neighbors = []#set()  # sets vermeiden Duplikate

        for dx in range(-threshold, threshold + 1):
            for dy in range(-threshold, threshold + 1):
                if dx == 0 and dy == 0:
                    continue  # überspringt current position
                #neighbors.add((x + dx, y + dy))  
                neighbors.append(Node((x + dx, y + dy)))
        return neighbors
        #return [Node(position) for position in neighbors]  # In Node-Objekte umwandeln und als Liste zurückgeben
    """
   
    def get_neighbors(self, threshold = 5): # muss 5 bleiben,um ganze "Kreuzung zu speichern"
        x, y = self.position
        #directions = [(threshold, 0), (-threshold, 0), (0, threshold), 
                      #(0, -threshold), (threshold, threshold), (-threshold, threshold), 
                      #(threshold, -threshold), (-threshold, -threshold)]  # Rechts, Links, Oben, Unten, Diagonale
        neighbors = []
        for i in range(threshold + 1):
            for j in range(threshold + 1): # "range(5) ist 0,1,2,3,4"
                if i == 0 and j == 0:
                    continue
                if i == 0 ^ j == 0: # "^" ist XOR
                    directions = [(j, i), (-j, i)]
                else:
                    directions = [(j, i), (-j, i),  
                            (j, -i), (-j, -i)]
                for dx, dy in directions:
                    new_x, new_y = x + dx, y + dy 
                    node = Node((new_x, new_y))
                    if node in neighbors:
                        continue
                    neighbors.append(node)
                    #rospy.loginfo(f"Nachbarn für {self.position}: {[n.position for n in neighbors]}")
                #if not neighbors:
                    #rospy.logwarn(f"Keine gültigen Nachbarn für {self.position}.")
        return neighbors
    
    def save_neighbors(self):
        self.neighbors = self.get_neighbors()

# Konstanten für Umrechnung und Geschwindigkeit
class Turtlebot3Explorer:
    def __init__(self):
        rospy.init_node('aufgabe2', anonymous=True)

        # SLAM und Kartierungsvariablen
        self.return_to_x = []  # Store X coordinates of blocked dead-ends
        self.return_to_y = []  # Store Y coordinates of blocked dead-ends
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        self.start_x = None
        self.start_y = None
        self.goal_x = -3.1363425254821777  #map2#Endpunkt must be changed
        self.goal_y = 3.001408338546753 #map2#Endpunkt must be changed
        self.linear_speed = 0.1 #meters
        self.backtraced_nodes = []
        self.laser_scan_msg = 0

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
        self.path_sub = rospy.Subscriber("/planned_path", PoseArray, self.path_callback)

    def odom_callback(self, msg: Odometry):
        """Update robot's current position and orientation from odometry."""
        """
        if self.start_x == None and self.start_y == None:
            self.start_x = msg.pose.pose.position.x
            self.start_y = msg.pose.pose.position.y
            node = Node((self.start_x, self.start_y))
            node.save_neighbors()
            self.backtraced_nodes.append(node)
            #self.visited_nodes.add(node)
            rospy.logwarn("Node hinzugefügt!")
        """
        if not self.map:
            rospy.logwarn("Warten auf SLAM.")
        else:
            self.current_x, self.current_y = self.real_to_pixel(msg.pose.pose.position.x, msg.pose.pose.position.y)
        #self.current_x= msg.pose.pose.position.x
        #self.current_y= msg.pose.pose.position.y
        self.current_yaw= msg.pose.pose.position.z
        
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
        goal_angle = math.atan2(goal_y - self.current_y, goal_x - self.current_x)
        angle_diff = goal_angle - self.current_yaw
        
        # Normalize angle to [-pi, pi]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
            
        return angle_diff
    
    def map_callback(self, msg: OccupancyGrid):

        if not msg.data or len(msg.data) == 0:
            rospy.logwarn("Leere oder ungültige SLAM-Daten empfangen. map_callback wird nicht ausgeführt.")
            return

        # Extrahiere Kartendaten und speichere sie
        self.map = msg
        self.map_resolution = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        #rospy.logerr(self.real_to_pixel(-1.5127006769180298,0.9310579895973206))
        
        # Konvertiere 1D-Kostendaten in eine 2D-Kostenkarte
        costmap_1d = msg.data
        self.costmap_2d = [costmap_1d[i:i + self.map_width] for i in range(0, len(costmap_1d), self.map_width)]
        rospy.loginfo("Map data converted to 2D costmap.")

        # Debug print for the robot's current position in the costmap
        robot_x, robot_y = self.real_to_pixel(self.current_x, self.current_y)
        if 0 <= robot_x < self.map_width and 0 <= robot_y < self.map_height:
            rospy.logwarn(f"Costmap value at robot position: {self.costmap_2d[robot_y][robot_x]}")
        

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

    
    def path_callback(self, msg: PoseArray):
        """Receive and store the temp_path."""
        self.temp_path = [(pose.position.x, pose.position.y) for pose in msg.poses]
        rospy.logwarn(self.temp_path)
        rospy.logwarn(f"Returning to: {self.temp_path[-1]} from: {self.temp_path[0]}")

    #Sirine
    def get_distance_to_goal(self, start_x, start_y, goal_x, goal_y):
        """Calculate Euclidean distance between two points."""
        return math.sqrt((goal_x - start_x)**2 + (goal_y - start_y)**2) / 100

    
    def get_distance_to_temp_goal(self, goal_x, goal_y):
        """Calculate Euclidean distance to the goal."""
        return math.sqrt((goal_x - self.current_x)**2 + (goal_y - self.current_y)**2)/100
    
    
    def move(self):
        cmd_vel = Twist()
        front_dist = self.distances.get('front', float('inf'))
        left_dist = self.distances.get('left', float('inf'))
        right_dist = self.distances.get('right', float('inf'))
        

        if  10 > left_dist > 0.4 or 10 > right_dist > 0.4:
            rospy.logwarn(f"right: {right_dist}, left: {left_dist}")
            #soufian
            #add the array to save the visited points
            #node = Node((self.real_to_pixel(self.current_x, self.current_y)))
            node = Node((self.current_x, self.current_y))
            if not self.is_node_in_backtraced_nodes(self.current_x, self.current_y): # "*" sorgt dafür, dass die Tupelwerte als separate argumente übergeben werden
                node.save_neighbors()
                self.backtraced_nodes.append(node)
                rospy.logwarn(f"Node hinzugefügt: {node.position}, Nachbarn: {[neighbor.position for neighbor in node.neighbors]}")
                rospy.logwarn(f"backtraced_nodes nach Hinzufügen: {[node.position for node in self.backtraced_nodes]}")
            
            cmd_vel.linear.x = 0
            direction = self.get_the_new_orientation() #the direction is used after 3 lines to define the direction of the rotation
            self.obstacle_timer = rospy.Time.now()
            while ((rospy.Time.now() - self.obstacle_timer).to_sec() < 2.4): # rotate about 90 grad            
                cmd_vel.angular.z = direction * 0.8 #you can adjust it to make sure that it rotate 90 grad
                self.cmd_vel_pub.publish(cmd_vel)
            #rospy.loginfo(f"direction ist :{direction}")
        
        cmd_vel.angular.z = 0
        cmd_vel.linear.x = self.linear_speed
        #rospy.loginfo(f"speed: {cmd_vel.linear.x}, distnace: {front_dist}")
        
        self.cmd_vel_pub.publish(cmd_vel)
        
        
        self.correct_the_position(left_dist,right_dist)

        if self.laser_scan_msg != 0:
            if self.check_closed_way():
                # Bedingung muss angepasst werden return_to_the_last_crossroads() soll nur aufgerufen werden, wenn es eine Sackgasse gibt 
                # bzw. der Robotererkennt, dass der Pfad ihn nicht weiterführt, weil er diesen schon abgeklappert hat 
                self.return_to_the_last_crossroads()
        
          
           
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
        #current_node = Node(((self.real_to_pixel(self.current_x, self.current_y))))#this is an instance
        current_node = Node((self.current_x, self.current_y))
        current_node.save_neighbors()
        #current_position = (self.current_x, self.current_y) this is just a tuple representing the robot position
        
        #get the neigbors point of the current_node
        #in the array 0: rechts, 1: links, 2: oben, 3: unten
        z = self.current_yaw
        if -0.75 < z <= -2.25: #the robot is looking at the up side
            right_neighbor = current_node.neighbors[0]  # Right neighbor
            left_neighbor = current_node.neighbors[1]  # Left neighbor
            front_neighbor = current_node.neighbors[2]  # front neighbor 
        elif -2.25 < z <= -3 or 2.25 < z <= 3: #the robot is looking at the left side
            right_neighbor = current_node.neighbors[2]  # Right neighbor
            left_neighbor = current_node.neighbors[3]  # Left neighbor
            front_neighbor = current_node.neighbors[1]  # front neighbor 
        elif 0.75 < z <= 2.25: #the robot is looking at the down side
            right_neighbor = current_node.neighbors[1]  # Right neighbor
            left_neighbor = current_node.neighbors[0]  # Left neighbor
            front_neighbor = current_node.neighbors[3]  # front neighbor
        else: #the robot is looking at the right side
            right_neighbor = current_node.neighbors[3]  # Right neighbor
            left_neighbor = current_node.neighbors[2]  # Left neighbor
            front_neighbor = current_node.neighbors[0]  # front neighbor


        # Get obstacle distances from LiDAR
        front_dist = self.distances.get('front', float('inf'))
        left_dist = self.distances.get('left', float('inf'))
        right_dist = self.distances.get('right', float('inf'))

        # Check if paths are clear
        front_clear = front_dist > 0.4
        left_clear = left_dist > 0.4
        right_clear = right_dist > 0.4

        # Check if neighbors have been visited
        right_visited = right_neighbor in self.backtraced_nodes
        left_visited = left_neighbor in self.backtraced_nodes
        front_visited = front_neighbor in self.backtraced_nodes 

        # Calculate the distance from each neighbor to the goal
        front_distance_to_goal = -1
        left_distance_to_goal = -1
        right_distance_to_goal = -1
         
        # Debugging logs
        #rospy.loginfo(f"Clear paths: Front: {front_clear}, Left: {left_clear}, Right: {right_clear}")
        #rospy.loginfo(f"Visited states: Front: {front_visited}, Left: {left_visited}, Right: {right_visited}")
        #rospy.loginfo(f"Distances to goal: Front: {front_distance_to_goal}, Left: {left_distance_to_goal}, Right: {right_distance_to_goal}")
        if front_clear and not front_visited:
            front_distance_to_goal = self.get_distance_to_goal(front_neighbor.position[0], front_neighbor.position[1], self.goal_x, self.goal_y)
        if right_clear and not right_visited:
            right_distance_to_goal = self.get_distance_to_goal(right_neighbor.position[0], right_neighbor.position[1], self.goal_x, self.goal_y)
        if left_clear and not left_visited:
            left_distance_to_goal = self.get_distance_to_goal(left_neighbor.position[0], left_neighbor.position[1], self.goal_x, self.goal_y)
        
        if left_distance_to_goal <= right_distance_to_goal:
            if left_distance_to_goal <= front_distance_to_goal:
                return -1
            else:
                return 0
        elif right_distance_to_goal <= front_distance_to_goal:
            return 1
        else:
            return 0
        

        '''
        # All three paths are clear
        if front_clear and left_clear and right_clear:
            # Check if all paths are visited
            if front_visited and left_visited and right_visited:
                rospy.logwarn("All paths (front, left, right) are already visited.")
                return None  # Or trigger backtracking if needed

            # If only front is visited
            if front_visited and not left_visited and not right_visited:
                return 1 if left_distance_to_goal < right_distance_to_goal else -1  
                # Choose between left and right

            # If only left is visited
            if left_visited and not front_visited and not right_visited:
                return 0 if front_distance_to_goal < right_distance_to_goal else -1  
                # Choose between front and right

            # If only right is visited
            if right_visited and not front_visited and not left_visited:
                return 0 if front_distance_to_goal < left_distance_to_goal else 1  
                # Choose between front and left

            # If front and left are visited, turn right
            if front_visited and left_visited and not right_visited:
                return -1  # Turn right

            # If front and right are visited, turn left
            if front_visited and right_visited and not left_visited:
                return 1  # Turn left

            # If left and right are visited, move forward
            if left_visited and right_visited and not front_visited:
                return 0  # Move forward

            # If none are visited, choose the shortest distance to the goal
            min_distance = min(front_distance_to_goal, left_distance_to_goal, right_distance_to_goal)
            if min_distance == front_distance_to_goal:
                return 0  # Move forward
            return 1 if min_distance == left_distance_to_goal else -1  # Turn left or right

        # Front and left are clear
        if front_clear and left_clear:
        
            # If front is visited, return left
            if front_visited and not left_visited:
                return 1  # Turn left

            # If left is visited, return front
            if left_visited and not front_visited:
                return 0  # Move forward

            # If neither is visited, compare distances
            if front_distance_to_goal < left_distance_to_goal:
                return 0  # Move forward 
            return 1  # Turn left

        # Front and right are clear
        if front_clear and right_clear:
            
            # If front is visited, return right
            if front_visited and not right_visited:
                return -1  # Turn right

            # If right is visited, return front
            if right_visited and not front_visited:
                return 0  # Move forward
            
            # If neither is visited, compare distances
            if front_distance_to_goal < right_distance_to_goal:
                return 0  # Move forward
            return -1  # Turn right
      
        # Left and right are clear
        if left_clear and right_clear:
        
            # If left is visited, return right
            if left_visited and not right_visited:
                return -1  # Turn right

            # If right is visited, return left
            if right_visited and not left_visited:
                return 1  # Turn left
 
            # If neither is visited, compare distances
            if left_distance_to_goal < right_distance_to_goal:
                return 1  # Turn left
            return -1  # Turn right

        # Only one path is clear
        if front_clear and not front_visited and not left_clear and not right_clear:
            return 0  # Move forward
        if left_clear and not left_visited and not front_clear and not right_clear:
            return 1  # Turn left
        if right_clear and not right_visited and not left_clear and not front_visited:
            return -1  # Turn right
        '''


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
        if len(self.backtraced_nodes) == 0:
            rospy.logwarn("Keine backtraced nodes vorhanden!")
        else:
            goal_node = self.backtraced_nodes.pop() # immernoch in der visited_nodes Liste als letztes Element abrufbar, falls man doch nochmal dorthin muss...
            rospy.logwarn(f"Backtraced Node: {goal_node.position}")
            self.publish_temp_map_and_points_to_a_star((self.current_x, self.current_y, 0.0), (goal_node.position[0], goal_node.position[1], 0.0)) # die a_star_planner node "triggert" auch den tb3_driver, der die Bewegung durchführt
            # Methode soll nicht vor Erreichen des goals verlassen werden
            while self.get_distance_to_temp_goal(goal_node.position[0], goal_node.position[1]) > 0.03: # 0.03 entspricht distance_threshold aus tb3_driver
                pass
        
    def real_to_pixel(self, real_x, real_y):
        """
        Konvertiert Real-World-Koordinaten (Meter) in Pixel-Koordinaten.
        """
        #rospy.loginfo(f"Converting real-world to pixel: Real=({real_x}, {real_y}), Origin=({self.map_origin_x}, {self.map_origin_y}), Resolution={self.map_resolution}")
        
        pixel_x = int((real_x - self.map_origin_x) / self.map_resolution)
        pixel_y = int((real_y - self.map_origin_y) / self.map_resolution)

        #rospy.loginfo(f"Resulting pixel coordinates: ({pixel_x}, {pixel_y})")
        return pixel_x, pixel_y
    
    def pixel_to_real(self, pixel_x, pixel_y):
        """
        Konvertiert Pixel-Koordinaten zurück in Real-World-Koordinaten (Meter).
        """
        real_x = (pixel_x * self.map_resolution) + self.map_origin_x
        real_y = (pixel_y * self.map_resolution) + self.map_origin_y
        return real_x, real_y
    
    def convert_2d_to_1d(self, map_2d):
        return [int(cell) for row in map_2d for cell in row]

    
    def publish_temp_map_and_points_to_a_star(self, temp_start: Point, temp_goal: Point):
        pkg = MapData()
        #modified_map_1d = self.convert_2d_to_1d(self.map_2d_modified, self.map_width, self.map_height)
        rospy.logwarn(f"Erster Wert in costmap_2d: {self.costmap_2d[0][0]}, Typ: {type(self.costmap_2d[0][0])}")

        pkg.costmap = self.convert_2d_to_1d(self.costmap_2d)
        pkg.resolution = self.map.info.resolution
        pkg.width = self.map.info.width
        pkg.height = self.map.info.height
        pkg.origin = Point(self.map.info.origin.position.x, self.map.info.origin.position.y, 0.0)

        if not self.map or not self.costmap_2d:
            rospy.logwarn("Map or costmap is not ready.")
            return

        #pixel_temp_start_x, pixel_temp_start_y = self.real_to_pixel(temp_start[0], temp_start[1])
        #pixel_temp_goal_x, pixel_temp_goal_y = self.real_to_pixel(temp_goal[0], temp_goal[1])
        pixel_temp_start_x, pixel_temp_start_y = temp_start[0], temp_start[1]
        pixel_temp_goal_x, pixel_temp_goal_y = temp_goal[0], temp_goal[1]

        if not (0 <= pixel_temp_start_x < self.map_width and 0 <= pixel_temp_start_y < self.map_height):
            rospy.logerr(f"Start position out of bounds: x={pixel_temp_start_x}, y={pixel_temp_start_y}")
            return

        if not (0 <= pixel_temp_goal_x < self.map_width and 0 <= pixel_temp_goal_y < self.map_height):
            rospy.logerr(f"Goal position out of bounds: x={pixel_temp_goal_x}, y={pixel_temp_goal_y}")
            return

        start_value = self.costmap_2d[pixel_temp_start_y][pixel_temp_start_x]
        goal_value = self.costmap_2d[pixel_temp_goal_y][pixel_temp_goal_x]

        rospy.logwarn(f"Start pixel value: {start_value}, Goal pixel value: {goal_value}")
        rospy.logwarn(f"Converting real-world to pixel: Pixel_Value=({pixel_temp_start_x}, {pixel_temp_start_y}), Map_Origin=({self.map_origin_x}, {self.map_origin_y}), Map_Resolution={self.map_resolution}")

        if start_value < 0 or goal_value < 0:
            rospy.logerr(f"Start or goal lies on an obstacle or unknown area: Start={start_value}, Goal={goal_value}")
            return


        self.temp_map_pub.publish(pkg)
        #self.temp_goal_pub.publish(Point(pixel_temp_goal_x, pixel_temp_goal_y, 0.0))
        #self.temp_start_pub.publish(Point(pixel_temp_start_x, pixel_temp_start_y, 0.0))

        self.temp_goal_pub.publish(Point(temp_goal[0], temp_goal[1], 0.0))
        self.temp_start_pub.publish(Point(temp_start[0], temp_start[1], 0.0))

    """
    def is_node_in_backtraced_nodes(self, curr_x, curr_y, tolerance=0.06):
        for node in self.backtraced_nodes:
            node_x, node_y = node.position
            # Berechne den Abstand zwischen aktueller Position und Node
            distance = math.sqrt((node_x - curr_x) ** 2 + (node_y - curr_y) ** 2)
            if distance <= tolerance:
                return True  # Node liegt im Bereich
        return False  # Keine passende Node gefunden
    """
    def is_node_in_backtraced_nodes(self, curr_x, curr_y):
        for node in self.backtraced_nodes:
            node_x, node_y = node.position
            if node_x == curr_x and node_y == curr_y:
                return True
        return False

    
                
                
   

    """
    def convert_2d_to_1d(self, map_2d, width, height):
        # Convert 2D map array back to 1D array
        map_1d = []
        for y in range(height):
            for x in range(width):
                map_1d.append(map_2d[y][x])
        return map_1d
    """



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
        for direction, angle_ranges in ANGLE_RANGES.items():
            for angle_range in angle_ranges:
                if not analyze_direction(angle_range):
                    #rospy.loginfo(f"Passage found in {direction} direction")
                    return False
        
        rospy.loginfo("All directions appear closed")
        return True
        

    #using the black points
    #needs SLAM
    #Danny
    '''
    def check_closed_way(self, current_x,current_y):
        """
        Check if the path is closed by analyzing nearby black points in the occupancy grid.
        """
        if self.map is None:
            return False  # Map not received yet
        rospy.loginfo("map received")

        MAX_GAP_THRESHOLD = 12  # cells (adjust based on your needs)
        SEARCH_RADIUS = 20  # cells to check around robot
        
        # Convert robot position to map coordinates
        robot_x = int((current_x - self.map_origin_x) / self.map_resolution)
        robot_y = int((current_y - self.map_origin_y) / self.map_resolution)
        
        # Check points in front, left, and right directions
        angles_to_check = [
            0,      # front
            90,     # left
            -90     # right
        ]
        
        for angle in angles_to_check:
            # Convert angle to radians
            angle_rad = math.radians(angle)
            
            # Look along this direction
            for dist in range(SEARCH_RADIUS):
                # Calculate point to check
                check_x = int(robot_x + dist * math.cos(angle_rad))
                check_y = int(robot_y + dist * math.sin(angle_rad))
                
                # Get cell value (if it's occupied)
                cell_index = check_y * self.map.info.width + check_x
                if 0 <= cell_index < len(self.map.data):
                    if self.map.data[cell_index] == 100:  # Found black point
                        break
                        
                # If we've gone too far without finding a wall
                if dist >= MAX_GAP_THRESHOLD:
                    rospy.loginfo(f"dist: {dist}")
                    return False
                    
        return True

    '''

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
            #when set the goal remove the commented next 3 lines
            #if self.get_distance_to_temp_goal(self.goal_x,self.goal_y) < 0.1:
                #self.stop_robot()
                #break
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