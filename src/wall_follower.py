#!/usr/bin/env python2

import numpy as np

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

        # to publish messages of type AckermannDriveStamped to the /drive topic
        self.publisher = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)

        # to subscribe to LIDAR data of type LaserScan
        rospy.Subscriber(self.SCAN_TOPIC, numpy_msg(LaserScan), self.laser_callback)

    # TODO:
    # Write your callback functions here.
    def laser_callback(self, data):
        # get an array of the distances from the lidar sensor to the nearest obstacle
        distances = np.array(data.ranges)

        # get the parameters for the follower
        desired_distance = self.DESIRED_DISTANCE
        velocity = self.VELOCITY
        side = self.SIDE

        # create a test message 
        test_ackermann = AckermannDriveStamped()
        test_ackermann.drive.speed = 1
        test_ackermann.drive.steering_angle = 1

        self.publisher.publish(test_ackermann)

        # TODO: detect a wall in a laser scan

        # TODO: use PD or PID control



if __name__ == "__main__":
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
    rospy.spin()
