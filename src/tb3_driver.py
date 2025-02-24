#!/usr/bin/env python3

import rospy
import math
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseArray
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker

class PathFollower:
    def __init__(self):
        rospy.init_node('controller')

        # Robot state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.path = None
        self.path_meter = None
        self.current_goal_index = 0
        self.save_path_once = False
        self.save_real_path_once = False

        self.goal_x = 0
        self.goal_y = 0

        self.path_sub_real = None
        self.path_sub = None
        self.already_oriented= False


        # Publishers and Subscribers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.path_sub_real = rospy.Subscriber('/real_world_path', PoseArray, self.real_path_callback)
        self.path_sub = rospy.Subscriber('/planned_path', PoseArray, self.path_callback)
        self.temp_goal_pub = rospy.Publisher('/temp_goal_pub', Marker, queue_size=10)
        
        # TF listener for coordinate transformations
        self.tf_listener = tf.TransformListener()



    def odom_callback(self, msg):
        """Update robot's current position and orientation from odometry."""
        self.current_x=-msg.pose.pose.position.x-0.75
        self.current_y=-msg.pose.pose.position.y+0.75
        
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


    def real_path_callback(self, msg):
        """Receive and store the planned path."""
        if not self.save_real_path_once:
            self.save_real_path_once= True
            self.path_meter = [(pose.position.x, pose.position.y) for pose in msg.poses]
            self.current_goal_index = 1


    def get_distance_to_goal(self):
            """Calculate Euclidean distance to the goal."""
            return math.sqrt((self.goal_x - self.current_x)**2 + (self.goal_y - self.current_y)**2)


    def get_angle_to_goal(self):
            """Calculate angle to the goal relative to robot's current heading."""
            dy = self.goal_y - self.current_y
            dx = self.goal_x - self.current_x
            
            # Invert the y-coordinates if your coordinate system has y increasing downwards
            goal_angle = math.atan2(dy, dx)
            angle_diff = goal_angle - self.current_yaw
            
            angle_diff_2 = (angle_diff) % (2 * math.pi) - math.pi
            
            #rospy.loginfo(f"yaw : {self.current_yaw}, goal_angle: {goal_angle}, ang_diff : {angle_diff}, {angle_diff_2}")
            
            return angle_diff_2
    
    def move_to_goal(self):
        """Generate velocity commands to move towards the current goal."""
        if not self.path or self.current_goal_index >= len(self.path):
            return self.stop_robot()
        
        self.goal_x , self.goal_y = self.path_meter[self.current_goal_index]

        # Calculate distance and angle to goal
        distance_to_goal = self.get_distance_to_goal()
        angle_diff = self.get_angle_to_goal()
        
        cmd_vel = Twist()
        
        goal_x_pixel, goal_y_pixel = self.path[self.current_goal_index]
        #self.check_line(goal_x_pixel,goal_y_pixel) #get the last point in the line (not point by point)
        #self.goal_x , self.goal_y = self.path_meter[self.current_goal_index] # because the index can be updated in the check_line Method

        marker = Marker()
        marker.header.frame_id = "map"  # Use the appropriate frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "temp_goals"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = self.goal_x
        marker.pose.position.y = self.goal_y
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
        
        # Check if we've reached the current waypoint
        if distance_to_goal < 0.23:
            self.current_goal_index += 1
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
                        self.current_goal_index = max(self.current_goal_index + counter -2,self.current_goal_index)
                        return
                else:
                    self.current_goal_index = max(self.current_goal_index + counter -2,self.current_goal_index)
                    return
                    
            counter = 1
            while next_goal_y == goal_y:
                counter += 1
                #rospy.loginfo(f"counter: {counter}")
                if self.current_goal_index + counter < len(self.path):
                    #rospy.loginfo(f"in2222222222")
                    _,next_goal_y = self.path[self.current_goal_index + counter]
                    if next_goal_y != goal_y:
                        self.current_goal_index = max(self.current_goal_index + counter -2,self.current_goal_index)
                        #rospy.loginfo(f"next_goal_y: {next_goal_y}, goal_y: {goal_y}, index : {self.current_goal_index} from y")
                        return
                else:
                    self.current_goal_index = max(self.current_goal_index + counter -2,self.current_goal_index)
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
            cmd_vel = self.move_to_goal()
            self.cmd_vel_pub.publish(cmd_vel)
            rate.sleep()

if __name__ == '__main__':
    try:
        controller = PathFollower()
        controller.run()
    except rospy.ROSInterruptException:
        pass