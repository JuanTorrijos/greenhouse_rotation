#!/usr/bin/env python3  

#Implementacion de avoid obstacles del profesor

from matplotlib.transforms import Transform
import rospy  
import numpy as np 
from sensor_msgs.msg import LaserScan   
from geometry_msgs.msg import Twist 
import tf
from tf2_geometry_msgs import PointStamped

# This class implements a simple obstacle avoidance algorithm 
class AvoidObstacleClass():  
    def __init__(self):  
        rospy.on_shutdown(self.cleanup) 

        ####################### PUBLISEHRS AND SUBSCRIBERS ############################  
        rospy.Subscriber("front/scan", LaserScan, self.laser_cb)  
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1) 

        ######################## CONSTANTS AND VARIABLES ##############################  
        self.laser_received = False 
        v_desired  = 0.4 #[m/s] desired speed when there are no obstacles 
        self.closest_angle = 0.0 #Angle to the closest object 
        self.closest_range = np.inf #Distance to the closest object 
        vel_msg = Twist() 

        # Tf Transform variables
        self.listener = tf.TransformListener()
        self.listener.waitForTransform("front_laser", "base_link", rospy.Time(0),rospy.Duration(4.0))
        self.lidar_point=PointStamped()
        self.lidar_point.header.frame_id = "front_laser"
        self.lidar_point.header.stamp =rospy.Time(0)
        # laser_point.point.x=0.0
        # laser_point.point.y=0.0
        # laser_point.point.z=0.0

        rate = rospy.Rate(10) #10Hz is the lidar's frequency  
        print("Node initialized 1hz") 

        ############################### MAIN LOOP ##################################### 
        while not rospy.is_shutdown():  
            ############################### YOUR CODE HERE ############################## 
            if self.laser_received: 
                self.laser_received = False 
                if np.isinf(min(self.lidar_msg.ranges)): #If there are no obstacles 
                    vel_msg.linear.x = 0.0 #Stop 
                    vel_msg.angular.z = 0.0 
                    print("Stoped")
                else: 
                    angles_array=[] 
                    x_array=[] 
                    y_array=[] 
                    for i in range(len(self.lidar_msg.ranges)): 
                        angle = self.get_angle(i, self.lidar_msg.angle_min, self.lidar_msg.angle_increment) 
                        r = self.lidar_msg.ranges[i] 
                        (x,y)=self.polar_to_cartesian(r,angle) 
                        x_array.append(x) 
                        y_array.append(y) 
                        angles_array.append(angle) 
                    xT=sum(x_array) 
                    yT=sum(y_array) 

                    transformedPoint = self.transform(xT,yT)
                    xT_robot = transformedPoint.point.x
                    yT_robot = transformedPoint.point.y
                    
                    thetaT=np.arctan2(yT_robot,xT_robot) 
                    dT=np.sqrt(xT_robot**2+yT_robot**2) 

                    print("xT lidar = ", xT)
                    print("yT lidar = ", yT)
                    print("xT robot = ", xT_robot)
                    print("yT robot = ", yT_robot)

                    # vel_msg.linear.x, vel_msg.angular.z = self.control_speed(dT, thetaT)  
                    print("Avoiding obstacle")                    

            self.cmd_vel_pub.publish(vel_msg) 
            rate.sleep()  
          
    def transform(self, x_lidar, y_lidar):
        self.lidar_point.point.x = x_lidar
        self.lidar_point.point.y = y_lidar
        p_jackal = self.listener.transformPoint("base_link", self.lidar_point)
        return p_jackal

    def laser_cb(self, msg):  
        ## This function receives a message of type LaserScan and computes the closest object direction and range 
        self.lidar_msg = msg 
        self.laser_received = True 

    def control_speed(self, distance, theta): 
        ## This function computes the linear and angular speeds for the robot 
        # given the distance and the angle theta as error references 
        kvmax = 0.3 #linear speed maximum gain 
        a = 1.0 #Constant to adjust the exponential's growth rate 
        kw = 0.9  # Angular speed gain  
        kv=kvmax*(1-np.exp(-a*distance**2))/distance #Constant to change the speed 
        v = kv * distance #linear speed 
        w = kw * theta #angular speed 
        return v, w 


    def get_angle(self, idx, angle_min, angle_increment): 
        ## This function returns the angle for a given element of the object in the lidar's frame 
        angle= angle_min + idx * angle_increment 
        # Limit the angle to [-pi,pi] 
        angle = np.arctan2(np.sin(angle),np.cos(angle)) 
        return angle 

    def polar_to_cartesian(self,r,theta): 
        ## This function converts polar coordinates to cartesian coordinates 
        if np.isinf(r): 
            r=self.lidar_msg.range_max 
        x = r*np.cos(theta) 
        y = r*np.sin(theta) 
        return (x,y) 
         

    def cleanup(self):  
        #This function is called just before finishing the node  
        # You can use it to clean things up before leaving  
        # Example: stop the robot before finishing a node.    
        vel_msg = Twist() 
        self.cmd_vel_pub.publish(vel_msg)  

############################### MAIN PROGRAM ####################################  
if __name__ == "__main__":  
    rospy.init_node("avoid_obstacle", anonymous=True)  
    AvoidObstacleClass() 