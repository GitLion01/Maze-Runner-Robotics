#!/usr/bin/env python3

import rospy 
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Point, PointStamped
from turtlebot_maze_navigation.msg import MapData
from sensor_msgs.msg import PointCloud,  PointCloud2,PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header


class RobotTracker:
    def __init__(self):
        rospy.init_node('odom_tracker', anonymous=True)
        self.current_position = None


        # Variables to store map data
        self.map = None
        self.map_2d = None # Placeholder for 2D costmap
        # Set a dummy goal point in map coordinates

        self.start_x = 0.03374290466308594
        self.start_y = -0.0013123006792739
        self.start_z = 0
        
        self.goal_x = None
        self.goal_y = None
        self.goal_z = 0.0

        self.width = None
        self.height = None

        # Flag to ensure odom_callback runs only once
        self.odom_processed = False

        self.checked_once =False

        # Subscribers to /odom and /map topics
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

        # Publisher 
        self.position_pub = rospy.Publisher('/robot_path', Point, queue_size=10)
        self.start_pub = rospy.Publisher('/start_point', Point, queue_size=10)
        self.goal_pub = rospy.Publisher('/goal_point', Point, queue_size=10)
        self.map_pub = rospy.Publisher('/map_data', MapData, queue_size=10)
        self.goal_sub = rospy.Subscriber('/maze_goal', PointStamped, self.set_goal)


    def set_goal(self, msg: Odometry):
        self.goal_x = msg.point.x
        self.goal_y = msg.point.y

        # Calculate the pixel coordinates of the goal
        pixel_goal_x = int((self.goal_x - self.map_origin_x) / self.map_resolution)
        pixel_goal_y = int((self.goal_y - self.map_origin_y) / self.map_resolution)
        
        # Check if the goal is within map bounds and in a free space
        if (0 <= pixel_goal_x < self.width and 0 <= pixel_goal_y < self.height and 
            self.map_2d[pixel_goal_y][pixel_goal_x] == 0):
            rospy.logwarn(f"Valid goal: x={self.goal_x}, y={self.goal_y}")
        else:
            rospy.logwarn(f"Valid goal: x={self.goal_x}, y={self.goal_y}")
            rospy.logwarn(f"Valid goal: x={self.goal_x}, y={self.goal_y}")
            rospy.logerr(f"Invalid goal position: map value={self.map_2d[pixel_goal_y][pixel_goal_x] if 0 <= pixel_goal_x < self.width and 0 <= pixel_goal_y < self.height else 'out of bounds'}")
            
        
        if self.map is not None and not self.checked_once:
            self.checked_once = True
            self.modify_the_map()          
        else:
            rospy.logwarn("Waiting for /map topic.")
        

    def odom_callback(self, msg):
        if self.map is not None:
            #Extracts the Robot position from the Odometry message
            position = msg.pose.pose.position 
            # Adjust for map origin and convert to pixel coordinates
            pixel_x = int(( (position.x) - self.map_origin_x) / self.map_resolution)
            pixel_y = int(( (position.y) - self.map_origin_y) / self.map_resolution)
            #convert the map/costmap from 1D to 2D ... 

            #Create a Point message with the pixel coordinates
            point = Point()
            point.x = pixel_x
            point.y = pixel_y
            point.z = 0  # Set z to 0 for 2D

            #Publish the robot's position in pixel coordinates
            self.position_pub.publish(point)
            #rospy.loginfo(f"Robot pixel position: x={pixel_x}, y={pixel_y}")

            # Update current position
            self.current_position = point
            #rospy.loginfo(f"Current position in pixels: x={point.x}, y={point.y}")
        else:
            rospy.logwarn("No map data received yet. Waiting for /map topic.")

    def convert_2d_to_1d(self, map_2d, width, height):
        """Convert 2D map array back to 1D array"""
        map_1d = []
        for y in range(height):
            for x in range(width):
                map_1d.append(map_2d[y][x])
        return map_1d
    
    def map_callback(self, msg):
        # Extract map data

        if self.map:
            return
      
        self.map = msg.info
        # Initialize map origin and resolution
        self.map_origin_x = self.map.origin.position.x
        self.map_origin_y = self.map.origin.position.y
        self.start_z = self.map.origin.position.z
        self.map_resolution = self.map.resolution
        self.width = self.map.width
        self.height = self.map.height

        # Convert the received 1D map to a 2D array
        self.map_2d = [msg.data[i:i + self.width] for i in range(0, len(msg.data), self.width)]


    def modify_the_map(self):
        # Create modified map with extended walls
        self.map_2d_modified = [list(row) for row in self.map_2d]
        wall_extension = 4  # Number of cells to extend walls
        self.add_100_to_the_next_points(self.height,self.width,wall_extension)
        
        # Convert modified 2D map back to 1D
        modified_map_1d = self.convert_2d_to_1d(self.map_2d_modified, self.width, self.height)

        pkg = MapData()
        pkg.costmap = modified_map_1d
        pkg.resolution = self.map.resolution
        pkg.width = self.map.width
        pkg.height = self.map.height
        pkg.origin = Point(self.map.origin.position.x, self.map.origin.position.y, 0.0)
        self.map_pub.publish(pkg)
        
        pixel_goal_x = int((self.goal_x - self.map_origin_x) / self.map_resolution)
        pixel_goal_y = int((self.goal_y - self.map_origin_y) / self.map_resolution)

        pixel_start_x = int((self.start_x - self.map_origin_x) / self.map_resolution)
        pixel_start_y = int((self.start_y - self.map_origin_y) / self.map_resolution)
        
        # Store the goal position in pixel coordinates
        gx, gy, gz = (pixel_goal_x, pixel_goal_y, 0.0)
        sx, sy, sz = (pixel_start_x, pixel_start_y, 0.0)

        self.goal_pub.publish(Point(gx, gy, gz))
        self.start_pub.publish(Point(sx, sy, sz))
        rospy.logwarn(f"pixel_goal: x={pixel_goal_x}, y={pixel_goal_y}")
        rospy.logwarn(f"pixel_start: x={pixel_start_x}, y={pixel_start_y}")

        rospy.logwarn(f"goal: x={self.goal_x}, y={self.goal_y}")
        rospy.logwarn(f"start: x={self.start_x}, y={self.start_y}")

        rospy.logwarn(f"map origin: x={self.map_origin_x}, y={self.map_origin_y}")
        rospy.logwarn(f"map resolution : {self.map_resolution}")

        rospy.logwarn(f"map height: {self.height}, width: {self.width}")

        self.publish_walls_after_edition()

    def add_100_to_the_next_points(self,height,width,wall_extension):
        # Extend walls
        for y in range(height):
            for x in range(width):
                if self.map_2d[y][x] == 100:  # If it's a wall
                    for dy in range(-wall_extension, wall_extension + 1):
                        for dx in range(-wall_extension, wall_extension + 1):
                            nx, ny = x + dx, y + dy
                            if 0 <= nx < width and 0 <= ny < height and not self.check_if_start_or_end_100(ny,nx):
                                if self.map_2d[ny][nx] in [-1, 0]:  # If target cell is unknown or free
                                    self.map_2d_modified[ny][nx] = 100
                            if self.check_if_start_or_end_100(ny,nx):
                                self.map_2d_modified[ny][nx] = 0

    def check_if_start_or_end_100(self,y,x):
        for x in range(x-1,x+2):
            for y in range(y-1,y+2):
                if (x == int((self.start_x - self.map_origin_x) / self.map_resolution) and y == int((self.start_y - self.map_origin_y) / self.map_resolution)) or \
                    (x == int((self.goal_x - self.map_origin_x) / self.map_resolution) and y == int((self.goal_y - self.map_origin_y) / self.map_resolution)):
                    return True
        return False
    
    def publish_walls_after_edition(self):
        '''
        sensor_msgs/PointCloud is needed,and not geometry_msgs/Point.
        '''
        wall_point_pub = rospy.Publisher('/extend_wall_points', PointCloud2, queue_size=10)
        point_cloud = PointCloud()
        point_cloud.header.frame_id = "map"
        point_cloud.header.stamp = rospy.Time.now()

        points = []
        for y in range(self.map.height):
            for x in range(self.map.width):
                if self.map_2d_modified[y][x] == 100:
                    world_x = self.map_origin_x + (x * self.map_resolution)
                    world_y = self.map_origin_y + (y * self.map_resolution)
                    points.append([world_x, world_y, 0.0])

        header = Header()
        header.frame_id = "map"
        header.stamp = rospy.Time.now()

        # Create PointCloud2 message
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1)
        ]

        # Publish the point cloud
        pc2_msg = pc2.create_cloud(header, fields, points)
        wall_point_pub.publish(pc2_msg)


if __name__ == '__main__':
    try:
        tracker = RobotTracker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass