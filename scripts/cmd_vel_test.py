#!/usr/bin/env python3
import rospy 
from std_msgs.msg import Int32 
from geometry_msgs.msg import Twist 
#This class will receive a number and an increment and it will publish the  
# result of adding number + increment in a recursive way. 
class Vel_class(): 
    def __init__(self): 
        rospy.on_shutdown(self.cleanup) 
        ###******* INIT PUBLISHERS *******### 
        ##  pub = rospy.Publisher('setPoint', UInt16MultiArray, queue_size=1) 
        self.pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1) 
        ############################### SUBSCRIBERS ##################################### 
        rospy.Subscriber("in_vel", Int32, self.vel_cb)
        ############ CONSTANTS ################ 
        self.vel_in = 0 #The velocity introduced by user 
        vel2pub = Twist()
        #********** INIT NODE **********### 
        r = rospy.Rate(10) #1Hz 
        print("Node initialized 10hz")
        while not rospy.is_shutdown(): 
            #self.vel_in=self.vel_in/100 #Convert velocity porcentage to real value
            vel2pub.linear.x = self.vel_in
            self.pub_vel.publish(vel2pub) #publish the number 
            print(vel2pub)
            r.sleep() 
        
    def vel_cb(self, vel): 
        ## This function receives a velocity porcentage.
        self.vel_in = vel.data/100
        pass 
        
    def cleanup(self): 
        #This function is called just before finishing the node 
        # You can use it to clean things up before leaving 
        # Example: stop the robot before finishing a node. 
        stop = Twist()
        self.pub_vel.publish(stop) #publish the number 
        print('\nProcess Terminated')
        pass 
############################### MAIN PROGRAM #################################### 
if __name__ == "__main__": 
    rospy.init_node("Vel_node", anonymous=True) 
    Vel_class() 