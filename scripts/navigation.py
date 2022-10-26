#!/usr/bin/env python3
from cmath import isinf
from random import randrange
from turtle import distance
from unicodedata import name
import rospy 
import numpy as np
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import os
import tf
from tf2_geometry_msgs import PointStamped


class navigation_nodeClass():
    def __init__(self):
        rospy.on_shutdown(self.cleanup) 
        ### Suscriber
        rospy.Subscriber("/scan_filtered", LaserScan, self.laser_sensor) 
        ### Publishers
        self.free_point_pub = rospy.Publisher("freepoint", PointStamped, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        ### Constants
        ### Variables
        # Tf Transform variables
        self.listener = tf.TransformListener()
        self.listener.waitForTransform("front_laser", "base_link", rospy.Time(0),rospy.Duration(4.0))
        self.lidar_point=PointStamped()
        self.lidar_point.header.frame_id = "front_laser"
        self.lidar_point.header.stamp =rospy.Time(0)
        vel_msg = Twist()
        self.listener = tf.TransformListener()
        self.listener.waitForTransform("front_laser", "base_link", rospy.Time(0),rospy.Duration(4.0))
        self.lidar_point=PointStamped()
        self.lidar_point.header.frame_id = "front_laser"
        self.lidar_point.header.stamp =rospy.Time(0)

        r = rospy.Rate(10)
        print('initialized node')
        while not rospy.is_shutdown():
            r.sleep()
    def polar2cartesian(self,angles,ranges):
        ranges = np.nan_to_num(ranges,posinf=25)
        ranges[ranges == 26] = 0
        #print(ranges)
        x = np.cos(angles)
        y = np.sin(angles)
        x = np.multiply(x,ranges)
        y = np.multiply(y,ranges)
        return x,y
    def cartesian2polar(self,x,y):
        x2 = x**2
        y2 = y**2
        angle = np.arctan(y/x)
        ranges = np.sqrt(x2+y2)
        return angle,ranges
    def transform(self, x_lidar, y_lidar):
        self.lidar_point.point.x = x_lidar
        self.lidar_point.point.y = y_lidar
        p_jackal = self.listener.transformPoint("base_link", self.lidar_point)
        return p_jackal  
    def laser_sensor(self,data):
        os.system('clear')
        print('data recieved')
        start = data.angle_min
        end = data.angle_max
        iterations = len(data.ranges)
        list_angles = np.linspace(start,end,iterations)
        list_ranges = np.array(data.ranges,dtype=np.float64)
        x,y = self.polar2cartesian(list_angles,list_ranges)
        free_point_x = np.sum(x)
        free_point_y = np.sum(y)
        print('X= ',free_point_x)
        print('Y= ',free_point_y)
        transformed_point = self.transform(free_point_x,free_point_y)
        print('New coordinates: ')
        print('X= ',transformed_point.point.x)
        print('Y= ',transformed_point.point.y)
        theta,radio = self.cartesian2polar(transformed_point.point.x,transformed_point.point.y)
        print('Angle: ',theta*180/np.pi)
        print('Radio: ',radio)
        return

    def transform(self, x_lidar, y_lidar):
        self.lidar_point.point.x = x_lidar
        self.lidar_point.point.y = y_lidar
        p_jackal = self.listener.transformPoint("base_link", self.lidar_point)
        return p_jackal
        
    def cleanup(self):
        return

### Main program

if __name__ == "__main__":
    rospy.init_node("navigation", anonymous=True)
    navigation_nodeClass()