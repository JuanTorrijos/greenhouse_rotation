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
        self.PV = 5e-4              # Linear velocity gain
        self.PW = 5                 # Angular velocity gain
        angle = 60
        self.weighted_limit = (angle+90)*np.pi/180    # Limit to apply the weighted range
        self.left_value = 25
        self.right_value = 30
        ### Variables
        self.list_angles = np.array
        self.list_ranges = np.array
        
        # Tf Transform variables
        self.listener = tf.TransformListener()
        self.listener.waitForTransform("front_laser", "base_link", rospy.Time(0),rospy.Duration(4.0))
        self.lidar_point=PointStamped()
        self.lidar_point.header.frame_id = "front_laser"
        self.lidar_point.header.stamp =rospy.Time(0)
        self.vel_msg = Twist()
        r = rospy.Rate(10)
        print('initialized node')
        while not rospy.is_shutdown():
            list_angles = self.list_angles
            list_ranges = self.list_ranges
            x,y = self.polar2cartesian(list_angles,list_ranges)
            free_point_x = np.sum(x)
            free_point_y = np.sum(y)
            print('X= ',round(free_point_x,2))
            print('Y= ',round(free_point_y,2))
            transformed_point = self.transform(free_point_x,free_point_y)
            print('New coordinates: ')
            print('X= ',round(transformed_point.point.x,2))
            print('Y= ',round(transformed_point.point.y,2))
            theta,radio = self.cartesian2polar(transformed_point.point.x,transformed_point.point.y)
            print('Angle: ',round(theta*180/np.pi,2))
            print('Radio: ',round(radio,2))
            self.navigation(theta,radio)

            r.sleep()
    def polar2cartesian(self,angles,ranges):
        print('Initial len angles: ',len(angles))
        print('Initial len ranges: ',len(ranges))
        angles = np.delete(angles,np.where(ranges == 26))
        ranges = np.delete(ranges,np.where(ranges == 26))
        print('Final len angles: ',len(angles))
        print('Final len ranges: ',len(ranges))
        print('Limite: ',self.weighted_limit,' angle: ',self.weighted_limit*180/np.pi)
        ranges[(angles<self.weighted_limit) & (angles>0)] = np.nan_to_num(ranges[(angles<self.weighted_limit) & (angles>0)],posinf=self.right_value)
        ranges = np.nan_to_num(ranges,posinf=self.left_value)
        #for i in range(len(angles)):
         #   print('Range: ',round(ranges[i],2),'Angle: ',round(angles[i]*180/np.pi,2))
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
    def navigation(self,theta,rho):
        lin_vel = rho * self.PV
        self.vel_msg.linear.x = lin_vel
        ang_vel = theta * self.PW
        self.vel_msg.angular.z = ang_vel
        print('Linear vel= ',round(lin_vel,2))
        print('Angular vel= ',round(ang_vel,2))
        self.cmd_vel_pub.publish(self.vel_msg)
        return 
    def laser_sensor(self,data):
        os.system('clear')
        print('data recieved')
        start = data.angle_min
        end = data.angle_max
        iterations = len(data.ranges)
        self.list_angles = np.linspace(start,end,iterations)
        self.list_ranges = np.array(data.ranges,dtype=np.float64)
        return 
    def cleanup(self):
        self.vel_msg.linear.x = 0
        self.vel_msg.angular.z = 0
        self.cmd_vel_pub.publish(self.vel_msg)
        print('###')
        print('Node killed successfully')
        print('###')
        return

### Main program

if __name__ == "__main__":
    rospy.init_node("navigation", anonymous=True)
    navigation_nodeClass()