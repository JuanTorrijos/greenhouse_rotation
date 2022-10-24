#!/usr/bin/env python3
from random import randrange
from turtle import distance
import rospy 
import numpy as np
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import os
import tf
from tf2_geometry_msgs import PointStamped
#This class will receive a laserScan and finds the closest object
class AvoidObstacleClass(): 
    def __init__(self): 
        rospy.on_shutdown(self.cleanup)  
        ############################### SUBSCRIBERS ##################################### 
        rospy.Subscriber("front/scan", LaserScan, self.laser_cb) 
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.free_point_pub = rospy.Publisher("freepoint", PointStamped, queue_size=1)
        vel_msg = Twist()
        ############ CONSTANTS ################ 
        self.closest_range = 0.0    # Distance to the closest object
        self.closest_angle = 0.0    # Angle to the closest object
        kw = 1.0                    # Angular velocity gain
        kv = 0.004                 # Desired linear speed
        self.thetaT = 0
        self.dt = 0
        self.xt = 0
        self.yt = 0

        # Tf Transform variables
        self.listener = tf.TransformListener()
        self.listener.waitForTransform("front_laser", "base_link", rospy.Time(0),rospy.Duration(4.0))
        self.lidar_point=PointStamped()
        self.lidar_point.header.frame_id = "front_laser"
        self.lidar_point.header.stamp =rospy.Time(0)

        #**** INIT NODE ****### 
        r = rospy.Rate(10) #10Hz is the lidar's frequency
        print("Node initialized 10hz")
        while not rospy.is_shutdown(): 
            range = self.closest_range
            theta_closest = self.closest_angle
            thetaAO = theta_closest - np.pi
            thetaAO = np.arctan2(np.sin(thetaAO), np.cos(thetaAO))
            #theta_closest = np.arctan2(np.sin(theta_closest),np.cos(theta_closest))

            # if np.isposinf(range): #If there are no obstacles
            #     range = 8.0

            os.system('clear')
            if np.isposinf(range): #If there are no obstacles
                print("No object detected")
                vel_msg.linear.x = 0.4
                vel_msg.angular.z = 0.0
            elif range <= 0.5: #if there is any obstacle in the range
                vel_msg.linear.x = 0.0
                vel_msg.angular.z = 0.0
                print("Object too close")
            elif range <= 1.0: #if there is any obstacle in the range
                vel_msg.linear.x = kv * self.dt
                vel_msg.angular.z = kw * self.thetaT
                print("Object in the range")
            else:
                print("No objects close")
                vel_msg.linear.x = 0.2
                vel_msg.angular.z = 0.0
            
            #os.system('clear')
            print("closest object distance: " + str(range))
            print("theta closest" + str(theta_closest))
            
            # point = self.coordinates(theta_closest, range)
            transformedPoint = self.transform(self.xt, self.yt)
            self.free_point_pub.publish(transformedPoint)
            xT_robot = transformedPoint.point.x
            yT_robot = transformedPoint.point.y

            self.thetaT = np.arctan2(yT_robot, xT_robot) # angle to the free point
            self.dt = np.sqrt((xT_robot*2)+(yT_robot*2))# Distance to the free point
            print("XT: ", xT_robot, "YT: ", yT_robot)
            print("ThetaT: ", self.thetaT, "Dt: ", self.dt)
            

            self.xt = 0.0
            self.yt = 0.0
            print("Vel: ", vel_msg)

            self.cmd_vel_pub.publish(vel_msg)
            r.sleep() 
            
    def laser_cb(self, msg): 
        ## This function receives a number
        #For this lidar
        self.closest_range = min(msg.ranges)
        idx = msg.ranges.index(self.closest_range)
        self.closest_angle = msg.angle_min + idx * msg.angle_increment
        # print("Closest object distance: " + str(self.closest_range))
        # print("Closest object direction: " + str(self.closest_angle))
        distances = msg.ranges
        for i in range(len(distances)):
            if i > 270 and i < 450:           # Because we put the wall in the back 1/4 of the lectures
                distance = 1.0
            else:
                distance = distances[i]             # This is like a back curved wall 
            # distance = distances[i]

            # if i < 270 or i > 450:
            #     distance = distances[i]
            
            if np.isposinf(distance): #If there are no obstacles
                distance = 8.0
            point = self.coordinates(msg.angle_min + i*msg.angle_increment, distance)
            self.xt += point[0]
            self.yt += point[1]
        # print("Indice maximo =", len(msg.ranges))
        
    def transform(self, x_lidar, y_lidar):
        self.lidar_point.point.x = x_lidar
        self.lidar_point.point.y = y_lidar
        p_jackal = self.listener.transformPoint("base_link", self.lidar_point)
        return p_jackal   
        
    def cleanup(self): 
        #This function is called just before finishing the node 
        # You can use it to clean things up before leaving 
        # Example: stop the robot before finishing a node.   
        vel_msg = Twist()
        self.cmd_vel_pub.publish(vel_msg)
        print("camara pues")
    
    def coordinates(self, angle, range):
        x = range * np.cos(angle)
        y = range * np.sin(angle)
        # print ("x: ", x)
        # print("y: ", y)
        point = [x,y]
        return point

############################### MAIN PROGRAM #################################### 

if __name__ == "__main__": 
    rospy.init_node("avoid_obstacle", anonymous=True) 
    AvoidObstacleClass()