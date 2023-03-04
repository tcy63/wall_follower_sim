#!/usr/bin/env python2

import numpy as np

import rospy
from std_msgs.msg import Float32
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_tools import *

class WallFollower:
    # Import ROS parameters from the "params.yaml" file.
    # Access these variables in class functions with self:
    # i.e. self.CONSTANT
    SCAN_TOPIC = rospy.get_param("wall_follower/scan_topic")
    DRIVE_TOPIC = rospy.get_param("/vesc/ackermann_cmd_mux/input/navigation")
    SIDE = rospy.get_param("wall_follower/side")
    VELOCITY = rospy.get_param("wall_follower/velocity")
    DESIRED_DISTANCE = rospy.get_param("wall_follower/desired_distance")
    FRONT_WALL_SETPOINT = 2.0
    WALL_TOPIC = "/wall"
    FRONT_WALL_TOPIC = "/front_wall"
    
    dt = 0.1
    prev_time = 0.0
    curr_time = 0.0
    error = 0.0
    prev_error = 0.0
    proportional = 0.0
    integral = 0.0
    derivative = 0.0
    Kp = 0.5
    Ki = 0.02
    Kd = 0.005
    deg2rad = np.pi / 180.0
    
    def __init__(self):
        # TODO:
        # Initialize your publishers and
        # subscribers here
        
        self.drive_pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)
        
        self.line_pub_1 = rospy.Publisher(self.WALL_TOPIC, Marker, queue_size=1)
        # self.line_pub_2 = rospy.Publisher(self.FRONT_WALL_TOPIC, Marker, queue_size=1)
        self.des_dist = rospy.Publisher('des_dist', Float32, queue_size=10)
        self.act_dist = rospy.Publisher('act_dist', Float32, queue_size=10)
        
        rospy.Subscriber(self.SCAN_TOPIC, numpy_msg(LaserScan), self.laser_callback) 
        
        self.prev_time = rospy.get_time()
        
    
    # TODO:
    # Write your callback functions here.
    def laser_callback(self, scan):
        sensed_dist, front_dist = self.filtered_sensor_data(scan)
        
        # self.dt = scan.time_increment
        self.curr_time = rospy.get_time()
        
        self.dt = self.curr_time - self.prev_time
        
        drive_command = AckermannDriveStamped()
        
        drive_steering_ang = self.PID_controller(sensed_dist, front_dist)
            
        drive_command.header.stamp = rospy.Time.now()
        drive_command.drive.speed = self.VELOCITY
        drive_command.drive.steering_angle = drive_steering_ang
            
        self.drive_pub.publish(drive_command)
            
        self.prev_time = rospy.get_time()
        
    def filtered_sensor_data(self, scan):
        if self.SIDE == -1:
            ang_count_min = int(((-100.0 * self.deg2rad) - scan.angle_min) / scan.angle_increment)
            ang_count_max = int(((-20.0 * self.deg2rad) - scan.angle_min) / scan.angle_increment)
            range_val_side = scan.ranges[ang_count_min:ang_count_max]
            ang_count_side = np.arange(ang_count_min, ang_count_max, 1)
        elif self.SIDE == 1:
            ang_count_min = int(((20.0 * self.deg2rad) - scan.angle_min) / scan.angle_increment)
            ang_count_max = int(((100.0 * self.deg2rad) - scan.angle_min) / scan.angle_increment)
            range_val_side = scan.ranges[ang_count_min:ang_count_max]
            ang_count_side = np.arange(ang_count_min, ang_count_max, 1)
        
        x_side = range_val_side * np.cos(scan.angle_min + scan.angle_increment * ang_count_side)
        y_side = range_val_side * np.sin(scan.angle_min + scan.angle_increment * ang_count_side)
        
        sd_y = np.std(y_side)
        sd_x = np.std(x_side)

        mean_y = np.mean(y_side)
        mean_x = np.mean(x_side)

        adj_y = []
        adj_x = []

        for ind_y, val in enumerate(y_side):
            if abs(val - mean_y) < 1.3 * sd_y and abs(x_side[ind_y] - mean_x) < 1 * sd_x:
                adj_y.append(val)
                adj_x.append(x_side[ind_y])

        adj_y = np.array(adj_y)
        adj_x = np.array(adj_x)
        
        fit_coeff_side = np.polyfit(adj_x, adj_y, 1)
        
        y_side_viz = (fit_coeff_side[0] * adj_x) + fit_coeff_side[1]
        
        ang_count_min = int(((-2.0 * self.deg2rad) - scan.angle_min) / scan.angle_increment)
        ang_count_max = int(((2.0 * self.deg2rad) - scan.angle_min) / scan.angle_increment)
        range_val_front = scan.ranges[ang_count_min:ang_count_max]
        ang_count_front = np.arange(ang_count_min, ang_count_max, 1)
    
        x_front = range_val_front * np.cos(scan.angle_min + scan.angle_increment * ang_count_front)
        y_front = range_val_front * np.sin(scan.angle_min + scan.angle_increment * ang_count_front)
        
        fit_coeff_front = np.polyfit(x_front, y_front, 1)
        
        y_front_viz = (fit_coeff_front[0] * x_front) + fit_coeff_front[1]
        
        VisualizationTools.plot_line(adj_x, y_side_viz, self.line_pub_1, frame="/laser")
        # VisualizationTools.plot_line(x_front, y_front_viz, self.line_pub_1, frame="/laser")
        
        sensed_dist = abs(fit_coeff_side[1] / np.sqrt(fit_coeff_side[0]**2 + 1))
        
        front_dist = abs(fit_coeff_front[1] / np.sqrt(fit_coeff_front[0]**2 + 1))
        # front_dist = scan.ranges[50]
        
        return sensed_dist, front_dist
    
    def PID_controller(self, sensed_dist, front_dist):
        # rospy.loginfo(sensed_dist)
        
        self.des_dist.publish(self.DESIRED_DISTANCE)
        self.act_dist.publish(sensed_dist)
        
        if front_dist < (self.FRONT_WALL_SETPOINT*self.DESIRED_DISTANCE):
            self.error = - self.SIDE * (((self.DESIRED_DISTANCE - sensed_dist) + (self.FRONT_WALL_SETPOINT*self.DESIRED_DISTANCE - front_dist))/2)
            # self.error = - self.SIDE * (self.DESIRED_DISTANCE - sensed_dist + 4.0)
        else:
            self.error = - self.SIDE * (self.DESIRED_DISTANCE - sensed_dist)
            
        # rospy.loginfo(self.error)
        
        self.proportional = self.error
        self.integral = self.integral + (self.error * self.dt)
        self.derivative = (self.error - self.prev_error) / self.dt
        
        drive_steering_ang = (self.Kp * self.proportional) + (self.Ki * self.integral) + (self.Kd * self.derivative)
        
        max_steering_ang = 0.34
        if drive_steering_ang > max_steering_ang:
            drive_steering_ang = max_steering_ang
        elif drive_steering_ang < -max_steering_ang:
            drive_steering_ang = -max_steering_ang
        
        self.prev_error = self.error
        
        return drive_steering_ang

if __name__ == "__main__":
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
    rospy.spin()
