#!/usr/bin/env python


import numpy as np
import math

import rospy
import tf
import tf2_ros
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_tools import *

"""
A helper function for visualizing a straight line
"""
def visualize(x, y, color=(0, 1, 0)):
    publisher = rospy.Publisher('/wall', Marker, queue_size=1)
    VisualizationTools.plot_line(x, y, publisher, color, frame="/laser")


class WallFollower:
    # Import ROS parameters from the "params.yaml" file.
    # Access these variables in class functions with self:
    # i.e. self.CONSTANT
    SCAN_TOPIC = rospy.get_param("wall_follower/scan_topic")
    DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic")
    SIDE = rospy.get_param("wall_follower/side")
    VELOCITY = rospy.get_param("wall_follower/velocity")
    DESIRED_DISTANCE = rospy.get_param("wall_follower/desired_distance")

    def __init__(self):
        # TODO:
        # Initialize your publishers and
        # subscribers here

        # to accumulate errors here
        self.errors = [0]
        self.kp = 2
        self.ki = 0
        self.kd = 8

        # to publish messages of type AckermannDriveStamped to the /drive topic
        self.publisher = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)
        
        # to subscribe to LIDAR data of type LaserScan
        rospy.Subscriber(self.SCAN_TOPIC, numpy_msg(LaserScan), self.laser_callback)

    # Write your callback functions here.
    def laser_callback(self, data):
        # create the message that is going to be sent out
        drive_msg = AckermannDriveStamped()

        # get an array of the distances from the lidar sensor to the nearest obstacle
        distances = np.array(data.ranges)
        angle_increment = data.angle_increment
        angle_min = data.angle_min
        angle_max = data.angle_max

        # compute an array of the angles 
        angles = np.arange(angle_min, angle_max, angle_increment)   # [angle_min, angle_max)

        # get the parameters for the follower
        desired_distance = self.DESIRED_DISTANCE
        velocity = self.VELOCITY
        side = self.SIDE

        # compute the x-y coordinates of all points
        points_x = np.multiply(distances, np.cos(angles))
        points_y = np.multiply(distances, np.sin(angles))
        
        # get the left/right and front regions
        if side == -1:
            wall_distances = distances[10:40]
            wall_points_x = points_x[10:40]
            wall_points_y = points_y[10:40]
        else:
            wall_distances = distances[60:90]
            wall_points_x = points_x[60:90]
            wall_points_y = points_y[60:90]
        
        front_points_x = points_x[45:55]
        front_points_y = points_y[45:55]
        front_distances = distances[45:55]

        # fit the line
        k, b = np.polyfit(wall_points_x, wall_points_y, 1)
        wall_points_y_fit = k * wall_points_x + b

        # visualize to check the fitting line
        visualize(wall_points_x, wall_points_y_fit, (1, 0, 0))

        # compute the car's distance from the line
        r = np.array([0, 0])
        p1 = np.array([wall_points_x[0], wall_points_y_fit[0]])
        p2 = np.array([wall_points_x[-1], wall_points_y_fit[-1]])

        distance = np.abs(np.cross(p1 - r, p2 - p1) / np.linalg.norm(p2 - p1))

        # PID control
        error = distance - desired_distance
        error *= side

        self.errors.append(error)
        proportional_error = error * self.kp
        integral_error = sum(self.errors) * self.ki
        derivative_error = (self.errors[-1] - self.errors[-2]) * self.kd
        angle = proportional_error + integral_error + derivative_error

        # deal with special states
        # special case 1: if it is going into a corner/front wall
        if np.mean(front_distances) < 2.8 * desired_distance:
            angle = -0.34 * side

        # if side == 1:
        #     if np.mean(wall_distances) > 3 * desired_distance:
        #         angle = 0.34
        # special case 2: if it is going to far from a wall
        if np.mean(wall_distances) > 3 * desired_distance:
            angle = 0.34 * side
        
        # publish the message
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.drive.steering_angle = max(min(angle, 0.34), -0.34)
        drive_msg.drive.speed = velocity

        self.publisher.publish(drive_msg)


if __name__ == "__main__":
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
    rospy.spin()
