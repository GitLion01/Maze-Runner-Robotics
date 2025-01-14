#!/usr/bin/env python3

import math
import numpy as np

class EllipseMovement:
    def __init__(self, corner_detection_threshold=0.5, min_corner_angle=0.5):
        """
        Initialize EllipseMovement for smooth corner navigation.
        
        :param corner_detection_threshold: Distance threshold to detect potential corner
        :param min_corner_angle: Minimum angle change to consider a path a corner
        """
        self.corner_detection_threshold = corner_detection_threshold
        self.min_corner_angle = min_corner_angle

    def is_corner(self, path, current_index):
        """
        Detect if the current path segment represents a corner.
        
        :param path: List of (x, y) coordinates representing the path
        :param current_index: Current index in the path
        :return: Boolean indicating if the current path segment is a corner
        """
        if current_index < 1 or current_index >= len(path) - 1:
            return False

        # Get three consecutive points
        prev_point = path[current_index - 1]
        curr_point = path[current_index]
        next_point = path[current_index + 1]

        # Calculate vectors between points
        vec1 = (curr_point[0] - prev_point[0], curr_point[1] - prev_point[1])
        vec2 = (next_point[0] - curr_point[0], next_point[1] - curr_point[1])

        # Calculate angle between vectors
        dot_product = vec1[0]*vec2[0] + vec1[1]*vec2[1]
        vec1_mag = math.sqrt(vec1[0]**2 + vec1[1]**2)
        vec2_mag = math.sqrt(vec2[0]**2 + vec2[1]**2)
        
        # Prevent division by zero
        if vec1_mag == 0 or vec2_mag == 0:
            return False

        cos_angle = dot_product / (vec1_mag * vec2_mag)
        angle = math.acos(max(min(cos_angle, 1), -1))

        return angle > self.min_corner_angle

    def generate_ellipse_path(self, start_point, mid_point, end_point, num_points=20):
        """
        Generate an elliptical path around a corner.
        
        :param start_point: Starting point before the corner
        :param mid_point: Corner point
        :param end_point: Ending point after the corner
        :param num_points: Number of points to generate in the elliptical path
        :return: List of (x, y) coordinates representing the elliptical path
        """
        # Calculate vectors
        vec1 = (mid_point[0] - start_point[0], mid_point[1] - start_point[1])
        vec2 = (end_point[0] - mid_point[0], end_point[1] - mid_point[1])

        # Normalize vectors
        vec1_mag = math.sqrt(vec1[0]**2 + vec1[1]**2)
        vec2_mag = math.sqrt(vec2[0]**2 + vec2[1]**2)
        
        # Determine ellipse parameters
        a = min(vec1_mag, vec2_mag) * 0.5  # semi-major axis
        b = a * 0.5  # semi-minor axis
        
        # Calculate rotation angle
        start_angle = math.atan2(vec1[1], vec1[0])
        end_angle = math.atan2(vec2[1], vec2[0])
        
        # Normalize angle difference
        angle_diff = end_angle - start_angle
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        # Generate elliptical path
        ellipse_path = []
        for t in np.linspace(0, 1, num_points):
            # Parametric equations of an ellipse
            angle = start_angle + angle_diff * t
            x = mid_point[0] + a * math.cos(angle) * math.cos(angle_diff) - b * math.sin(angle) * math.sin(angle_diff)
            y = mid_point[1] + a * math.cos(angle) * math.sin(angle_diff) + b * math.sin(angle) * math.cos(angle_diff)
            ellipse_path.append((x, y))
            
        return ellipse_path

    def modify_path_around_corner(self, path, current_index):
        """
        Modify the path to include an elliptical trajectory around a corner.
        
        :param path: Original path
        :param current_index: Index of the corner point
        :return: Modified path with elliptical corner navigation
        """
        if current_index < 1 or current_index >= len(path) - 1:
            return path

        start_point = path[current_index - 1]
        mid_point = path[current_index]
        end_point = path[current_index + 1]

        # Generate elliptical path around the corner
        ellipse_path = self.generate_ellipse_path(start_point, mid_point, end_point)

        # Replace the corner points with the elliptical path
        modified_path = path[:current_index-1] + ellipse_path + path[current_index+1:]
        
        return modified_path
#to modify the corner
# ellipse_nav = EllipseMovement()
# if ellipse_nav.is_corner(self.path, self.current_goal_index):
#     self.path = ellipse_nav.modify_path_around_corner(self.path, self.current_goal_index)