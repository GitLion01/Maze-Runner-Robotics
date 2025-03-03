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
        self.path_meter = None
        self.current_goal_index = 0
        self.save_real_path_once = False

        self.goal_x = 0
        self.goal_y = 0

        self.path_sub_real = None
        self.already_oriented= False


        # Publishers and Subscribers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.path_sub_real = rospy.Subscriber('/real_world_path', PoseArray, self.real_path_callback)
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
        if not self.path_meter:
            return self.stop_robot()
        
        self.goal_x , self.goal_y = self.path_meter[self.current_goal_index]

        # Calculate distance and angle to goal
        distance_to_goal = self.get_distance_to_goal()
        angle_diff = self.get_angle_to_goal()
        
        cmd_vel = Twist()

        
        #else:
        cmd_vel.linear.x = 0.22

        if abs(angle_diff) > 0.6:
            cmd_vel.angular.z = 2.84 if angle_diff > 0 else -2.84
        elif abs(angle_diff) > 0.4:
            cmd_vel.angular.z = 2 if angle_diff > 0 else -2
        elif abs(angle_diff) > 0.3:
            cmd_vel.angular.z = 1.8 if angle_diff > 0 else -1.8
        elif abs(angle_diff) > 0.2:
            cmd_vel.angular.z = 1.5 if angle_diff > 0 else -1.5
        elif abs(angle_diff) > 0.1:
            cmd_vel.angular.z = 0.1 if angle_diff > 0 else -0.1
        elif abs(angle_diff) > 0.05:
            cmd_vel.angular.z = 0.05 if angle_diff > 0 else -0.05
        else:
            cmd_vel.angular.z = 0
                
        
        # Check if we've reached the current waypoint
        if distance_to_goal < 0.22:
            if self.current_goal_index +1 < len(self.path_meter):
                self.current_goal_index += 1
            
        return cmd_vel
    
    
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