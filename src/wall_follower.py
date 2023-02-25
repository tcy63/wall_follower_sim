#!/usr/bin/env python


import numpy as np
import math

import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_tools import *

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
        self.kp = 10
        self.ki = 0
        self.kd = 0.1

        # to publish messages of type AckermannDriveStamped to the /drive topic
        self.publisher = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)

        # to subscribe to LIDAR data of type LaserScan
        rospy.Subscriber(self.SCAN_TOPIC, numpy_msg(LaserScan), self.laser_callback)

    # TODO:
    # Write your callback functions here.
    def laser_callback(self, data):
        # create the message that is going to be sent out
        drive_msg = AckermannDriveStamped()

        # get an array of the distances from the lidar sensor to the nearest obstacle
        distances = np.array(data.ranges)
        angle_increment = data.angle_increment
        angle_min = data.angle_min
        angle_max = data.angle_max

        # get the parameters for the follower
        desired_distance = self.DESIRED_DISTANCE
        velocity = self.VELOCITY
        side = self.SIDE

        # TODO: detect a wall in a laser scan
        # slice the ranges to get [-\pi/2,0] if following the left wall and [0, \pi/2] if right wall
        left_index = 0
        right_index = len(distances)
        wall_angle_min = -math.pi/2
        wall_angle_max = math.pi/2

        if side == 1:  # if the car is following the left wall
            wall_angle_min = max(angle_min, wall_angle_min)
            wall_angle_max = min(angle_max, 0)
            if angle_min < -math.pi/2:
                left_index = (-math.pi/2 - angle_min) / angle_increment
            if angle_max >= 0:
                right_index = (0 - angle_min) / angle_increment
        else:   # the car is follwing the right wall
            wall_angle_min = max(angle_min, 0)
            wall_angle_max = min(angle_max, wall_angle_max)
            if angle_min < 0:
                left_index = (0 - angle_min) / angle_increment
            if angle_max > math.pi/2:
                right_index = (math.pi/2 - angle_min) / angle_increment
        
        wall_distances = distances[left_index:right_index+1]    # from wall_angle_min to wall_angle_max with an increment of angle_increment
        
        # take tha range data into (x,y) points in the car's local coordinate systems
        wall_x = [wall_distances[i] * np.cos(wall_angle_min + i * angle_increment) for i in len(wall_distances)]
        wall_y = [wall_distances[i] * np.sin(wall_angle_min + i * angle_increment) for i in len(wall_distances)]
        
        # fit a line to these points by least square linear regression
        A = np.hstack(np.ones_like(wall_x), wall_x)
        Y = wall_y
        X = np.linalg.inv(np.transpose(A) @ A) @ np.transpose(A) @ Y

        # plot the estimated wall and calculate the car's distance from that wall
        distance = X[0] / np.sqrt(np.power(X[1]) + 1)

        # TODO: use PD or PID control

        error = desired_distance - distance

        self.errors.append(error)
        proportional_error = error * self.kp
        integral_error = sum(self.errors) * self.ki
        derivative_error = (self.errors[-1] - self.errors[-2]) * self.kd
        angle = proportional_error + integral_error + derivative_error

        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = velocity
        self.publisher.publish(drive_msg)


if __name__ == "__main__":
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
    rospy.spin()
