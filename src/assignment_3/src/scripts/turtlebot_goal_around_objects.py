#!/usr/bin/env python

import rospy
import tf
import numpy
import geometry_msgs.msg
from geometry_msgs.msg import Twist
from math import *
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import tf2_ros

class turtlebot_goal_around_objects():

    def __init__(self):
        # initiliaze
        rospy.init_node('turtlebot_goal_around_objects', anonymous=True)
        rospy.on_shutdown(self.shutdown)
        # Obstacle Detection Stuff
        self.pub = rospy.Publisher('/cmd_vel_mux/input/navi',Twist, queue_size=10)
        self.sub = rospy.Subscriber('/scan', LaserScan, self.call_back)
        self.sect_1 = 0
        self.sect_2 = 0
        self.sect_3 = 0
        # self.sect = 0
        self.ang = {0:0,1:-.5,10:-.5,11:-.5,100:.5,101:0,110:.5,111:-.5}
        self.fwd = {0:0,1:0,10:0,11:0,100:0,101:.1,110:0,111:0}
        self.dbgmsg = {0:'Do',1:'Veer right',10:'Veer right',11:'Veer right',100:'Veer left',101:'Veer left',110:'Veer left',111:'Veer right'}
        # Move Goal Stuff
        self.turtlebot_odom_pose = Odometry()
        pose_message = Odometry()
        self.target = Odometry()
        self.target.pose.pose.position.x = -4            # This is the target x position (change me)
        self.target.pose.pose.position.y = -4            # This is the target y position (change me)
        self.target.pose.pose.position.z = 0 
        self.target.pose.pose.orientation.w = 1
        self.target.pose.pose.orientation.x = 0
        self.target.pose.pose.orientation.y = 0
        self.target.pose.pose.orientation.z = 0
        self.velocityPublisher = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)  
        r = rospy.Rate(20)
        r.sleep()
        tolerance = .05
        while(self.getDistance(self.turtlebot_odom_pose.pose.pose.position.x, self.turtlebot_odom_pose.pose.pose.position.y, self.target.pose.pose.position.x, self.target.pose.pose.position.y) > tolerance):
            if(self.sect_1 + self.sect_2 + self.sect_3 == 0 ):
                self.moveGoal(tolerance)
            if(self.sect_1 + self.sect_2 + self.sect_3 > 0):
                self.movement(self.sect_1, self.sect_2, self.sect_3)
        VelocityMessage = Twist()
        VelocityMessage.linear.x = 0
        VelocityMessage.angular.z = 0
        self.velocityPublisher.publish(VelocityMessage)  
        print('Arrived at Goal! =]')
        print('The node is now ending...') 

    def call_back(self,laserscan):
        self.sort(laserscan)

    def poseCallback(self,pose_message):
        self.turtlebot_odom_pose.pose.pose.position.x=pose_message.pose.pose.position.x
        self.turtlebot_odom_pose.pose.pose.position.y=pose_message.pose.pose.position.y
        self.turtlebot_odom_pose.pose.pose.position.z=pose_message.pose.pose.position.z
        self.turtlebot_odom_pose.pose.pose.orientation.w=pose_message.pose.pose.orientation.w
        self.turtlebot_odom_pose.pose.pose.orientation.x=pose_message.pose.pose.orientation.x
        self.turtlebot_odom_pose.pose.pose.orientation.y=pose_message.pose.pose.orientation.y
        self.turtlebot_odom_pose.pose.pose.orientation.z=pose_message.pose.pose.orientation.z 
    
    def quat2Euler(self, q):
        t3 = 2.0 * (q.w * q.z + q.x * q.y)
        t4 = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = atan2(t3,t4)
        return yaw

    def calculateYaw(self, x1, y1, x2,y2):
        bearing = atan2((y2 - y1),(x2 - x1))
        return bearing
   
    def getDistance(self, x1, y1, x2, y2):
        return (sqrt((x1-x2)**2 + (y1-y2)**2))

    def moveGoal(self, tolerance):
        self.pose_subscriber = rospy.Subscriber("/odom", Odometry, self.poseCallback)
        x1 = self.turtlebot_odom_pose.pose.pose.position.x
        y1 = self.turtlebot_odom_pose.pose.pose.position.y
        x2 = self.target.pose.pose.position.x
        y2 = self.target.pose.pose.position.y
        VelocityMessage = Twist()
        error_x = 0
        error_prev_x = 0
        der_error_x = 0
        int_error_x = 0
        kp_x = 0.2
        ki_x = 0.005
        kd_x = 0.01
        error_th = 0
        error_prev_th = 0
        der_error_th = 0
        int_error_th = 0
        kp_th = 3
        ki_th = 0.1
        kd_th = 0.1
        max_x = .2
        max_th = .2
        dt = 20
        r = rospy.Rate(dt)
        # X control
        error_prev_x = error_x
        error_x = self.getDistance(x1, y1, x2, y2)
        der_error_x = (error_x - error_prev_x)/dt
        int_error_x = (error_x - error_prev_x)*dt/2
        VelocityMessage.linear.x = kp_x*error_x + ki_x*int_error_x + kd_x*der_error_x
        # Theta control
        error_prev_th = error_th
        error_th = self.calculateYaw(x1, y1, x2, y2) - self.quat2Euler(self.turtlebot_odom_pose.pose.pose.orientation)
        der_error_th = (error_th - error_prev_th)/dt
        int_error_th = (error_th - error_prev_th)*dt/2
        VelocityMessage.angular.z = kp_th*error_th + ki_th*int_error_th + kd_th*der_error_th
        # Cap the speeds
        if VelocityMessage.linear.x > max_x:
            VelocityMessage.linear.x = max_x
        if VelocityMessage.linear.x < -max_x:
            VelocityMessage.linear.x = -max_x
        if VelocityMessage.angular.z > max_th:
            VelocityMessage.angular.z = max_th
        if VelocityMessage.angular.z < -max_th:
            VelocityMessage.angular.z = -max_th
        self.velocityPublisher.publish(VelocityMessage)
        # where are we?
        self.pose_subscriber = rospy.Subscriber("/odom", Odometry, self.poseCallback)
        x1 = self.turtlebot_odom_pose.pose.pose.position.x
        y1 = self.turtlebot_odom_pose.pose.pose.position.y
        print('I am moving toward the goal')
        r.sleep()
        
    def shutdown(self):
        self.velocityPublisher.publish(Twist())
        rospy.sleep(1)

    def reset_sect(self):      
	self.sect_1 = 0
	self.sect_2 = 0
	self.sect_3 = 0

    def sort(self, laserscan):
	    entries = len(laserscan.ranges)
	    for entry in range(0,entries):
	        if 0.05 < laserscan.ranges[entry] < 1:
		    self.sect_1 = 1 if (0 < entry < entries/3) else 0 
		    self.sect_2 = 1 if (entries/3 < entry < entries/2) else 0
		    self.sect_3 = 1 if (entries/2 < entry < entries) else 0

    def movement(self, sect1, sect2, sect3):
        dt = 2
        r = rospy.Rate(dt)
        VelocityMessage = Twist()
        sect = 1
        while(sect > 0):
            sect = int(str(self.sect_1) + str(self.sect_2) + str(self.sect_3))
            print('Object detected in section[s] ',sect)
            VelocityMessage.angular.z = self.ang[sect]
            VelocityMessage.linear.x = self.fwd[sect]
            self.velocityPublisher.publish(VelocityMessage)
            self.reset_sect()
            r.sleep()
        VelocityMessage.angular.z = 0
        VelocityMessage.linear.x = 0
        self.velocityPublisher.publish(VelocityMessage)        

if __name__ == '__main__':
    try:
        turtlebot_goal_around_objects()
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
