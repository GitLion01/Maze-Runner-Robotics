#!/usr/bin/env python3

import heapq
import rospy
from maze_bot_control.msg import MapData
from geometry_msgs.msg import Pose, PoseArray, Point

class Node:
    def __init__(self, position, parent=None):
        self.position = position
        self.parent = parent
        self.prev = None
        self.g_cost = 0.0
        self.h_cost =0.0
        self.f_cost = 0.0
        self.neighbors = []
        self.is_diagonal = False

    def __eq__(self, other):
        return self.position == other.position

    def __hash__(self):
        return hash(self.position)

    def __lt__(self, other): # "lt" = "less than"
        return self.f_cost < other.f_cost

    def get_neighbors(self, grid, closed_list):
        x, y, z = self.position
        directions = [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (1, 1, 0), (-1, 1, 0), (1, -1, 0), (-1, -1, 0)]  # Rechts, Links, Oben, Unten, Diagonale
        neighbors = []
        visited_positions = set()
        for dx, dy, _ in directions:
            new_x, new_y = x + dx, y + dy 
            # Überprüfen, ob der Nachbar im Grid ist und kein Hindernis darstellt
            if 0 <= int(new_y) < len(grid) and 0 <= int(new_x) < len(grid[0]) and grid[int(new_y)][int(new_x)] == 0:
                node = Node((new_x, new_y, 0.0))
                if node not in closed_list: 
                    visited_positions.add((node.position[0], node.position[1]))
                    neighbors.append(node)
        self.neighbors = neighbors
        


class AStar:
    def __init__(self, start: Node, goal: Node, grid: list, heuristic: callable):
        self.start = start
        self.goal = goal
        self.grid = grid
        self.heuristic = heuristic
        self.open_list = []
        self.closed_list = set()

    def reconstruct_path(self, current: Node):
        rospy.logwarn("Aufruf reconstruct_path().")
        path = []
        while current:
            path.append(current.position)
            current = current.parent
        
        path.reverse()  # Pfad umkehren

        rospy.logwarn(f"path reconstructed {len(path)}")
        return path

    def run(self):
        rospy.logwarn("Aufruf run()")
        # Initialisierung der Startwerte
        self.start.g_cost = 0.0
        self.start.h_cost = self.heuristic(self.start.position, self.goal.position)
        self.start.f_cost = self.start.g_cost + self.start.h_cost
        

        heapq.heappush(self.open_list, (self.start.f_cost, self.start))

        while self.open_list:
            _, current = heapq.heappop(self.open_list)  # Knoten mit niedrigstem f-Wert

            self.closed_list.add(current)
            if current.position == self.goal.position:
                    #rospy.logwarn("goal reached")
                    path = self.reconstruct_path(current)
                    #rospy.logwarn(path)
                    return path

            # Nachbarn generieren
            current.get_neighbors(self.grid, self.closed_list)

            for neighbor in current.neighbors:
                if neighbor in self.closed_list:
                    continue
                
                

                cost_neighbor = current.g_cost + (14.0 if current.is_diagonal else 10.0)  

                if neighbor.f_cost < current.f_cost or neighbor not in [n for _, n in self.open_list]:
                    # den neighbor nehmen, der uns näher zum ziel bringt
                    neighbor.g_cost = cost_neighbor
                    neighbor.h_cost = self.heuristic(neighbor.position, self.goal.position)
                    neighbor.f_cost = neighbor.g_cost + neighbor.h_cost
                    neighbor.f_cost = neighbor.g_cost + neighbor.h_cost
                    neighbor.parent = current
                    


                # Nachbarn in die Open List einfügen, wenn noch nicht vorhanden
                if neighbor not in [n for _, n in self.open_list]:
                    heapq.heappush(self.open_list, (neighbor.f_cost, neighbor))
        rospy.logwarn("End run()")
       
        return None  # Kein Pfad gefunden


class AStarNode:
    def __init__(self):
        rospy.init_node("a_star_planner")

        # Subscribes
        self.map_sub = rospy.Subscriber("/map_data", MapData, self.map_callback)
        self.goal_sub = rospy.Subscriber("/goal_point", Point, self.goal_callback)
        self.start_sub = rospy.Subscriber("/start_point", Point, self.start_callback)


        # Publisher
        self.real_world_path = rospy.Publisher("/real_world_path", PoseArray, queue_size=10)

        # Variablen
        self.grid = None
        self.path = None

        self.start_pixel = None
        self.goal_pixel = None
        self.map_resolution = None
        self.map_origin = None
        self.map_1D = None


    def goal_callback(self, msg: Point):
        if self.goal_pixel:
            return
        else:
            self.goal_pixel = (msg.x, msg.y, msg.z)
    
    def start_callback(self, msg: Point):
        if self.start_pixel:
            return
        else:
            self.start_pixel = (msg.x, msg.y, msg.z)
    
    def map_callback(self, msg: MapData):
        #rospy.logwarn("map_callback aufgerufen")

        if self.grid:  # Überprüfen, ob die Karte bereits gesetzt wurde
            return
        
        self.map_resolution = msg.resolution
        self.map_origin = (msg.origin.x, msg.origin.y, msg.origin.z) 
        self.map_1D = msg.costmap
        
        # Konvertiere Karte in ein 2D-Array
        width = msg.width
        height = msg.height
        data = list(msg.costmap)
        self.grid = [data[i:i + width] for i in range(0, len(data), width)]
        if self.grid and self.start_pixel:
            rospy.logwarn(f"Grid erfolgreich erstellt mit Dimensionen: {width}x{height}")
            rospy.logwarn(f"Wert der Startposition im Grid: {self.grid[int(self.start_pixel[1])][int(self.start_pixel[0])]}")
   
    def plan_path(self):
        """
        Führt den A*-Algorithmus aus und veröffentlicht den Pfad.
        """
        if not (self.grid and self.start_pixel and self.goal_pixel):
            rospy.logwarn("Unvollständige Daten: Karte, Start oder Ziel fehlen!")
            return
        

        if int(self.start_pixel[1]) >= len(self.grid) or int(self.start_pixel[0]) >= len(self.grid[0]):
            rospy.logerr(f"Ungültige Startposition: {self.start_pixel}")
            return
        if int(self.goal_pixel[1]) >= len(self.grid) or int(self.goal_pixel[0]) >= len(self.grid[0]):
            rospy.logerr(f"Ungültige Zielposition: {self.goal_pixel}")
            return
        
        # Überprüfe, ob Start- und Zielpunkte valide sind
        start_value = self.grid[int(self.start_pixel[1])][int(self.start_pixel[0])]
        goal_value = self.grid[int(self.goal_pixel[1])][int(self.goal_pixel[0])]
        if start_value != 0 or goal_value != 0:
            rospy.logerr(f"Start oder Ziel liegen auf Hindernissen: Start={start_value}, Ziel={goal_value}, goal_pixel_y ={self.goal_pixel[0]}, goal_pixel_x = {self.goal_pixel[1]}")
            return  # Abbrechen, wenn ungültig
        

        # Führe A* aus
        astar = AStar(Node(self.start_pixel), Node(self.goal_pixel), self.grid, manhatten_heuristic)
        self.path = astar.run()

        if self.path:
            rospy.loginfo("Pfad gefunden!")
            self.publish_real_world_path(self.path)
        else:
            rospy.logwarn("Kein Pfad berechnet.")
            self.path=None


    def publish_real_world_path(self, path):
        """
        Veröffentlicht den geplanten Pfad als PoseArray in Weltkoordinaten.
        """
        if not (self.map_resolution and self.map_origin):
            rospy.logerr("Map resolution or origin not set. Cannot convert pixel to world coordinates.")
            return

        path_msg = PoseArray()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "map"

        for px, py, pz in path:
            # Convert pixel coordinates to world coordinates
            world_x = self.map_origin[0] + (px * self.map_resolution)
            world_y = self.map_origin[1] + (py * self.map_resolution)

            pose = Pose()
            pose.position.x = world_x
            pose.position.y = world_y
            pose.position.z = pz  # Typically 0.0 for 2D paths
            pose.orientation.w = 1.0  # No orientation information for waypoints
            path_msg.poses.append(pose)

        self.real_world_path.publish(path_msg)
        rospy.loginfo(f"{self.real_world_path}")

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            if self.grid and self.start_pixel and self.goal_pixel:
                #rospy.loginfo("Alle Bedingungen erfüllt, starte plan_path().")
                self.plan_path()
                break
            #else:
                #rospy.logwarn(f"Bedingungen nicht erfüllt: Grid: {bool(self.grid)}, Start: {self.start_pixel}, Goal: {self.goal_pixel}")
            if self.path:
                self.publish_real_world_path(self.path)
            rate.sleep()



def manhatten_heuristic(node_position, goal_position):
    x1, y1, _ = node_position
    x2, y2, _ = goal_position
    return abs(x1 - x2) + abs(y1 - y2)

if __name__ == "__main__":
    try:
        node = AStarNode()
        node.run()
        rospy.spin()
    except Exception as e:
        rospy.logerr(f"Error during A* execution: {e}")

