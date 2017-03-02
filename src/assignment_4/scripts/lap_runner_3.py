#!/usr/bin/env python
import rospy
import math
import numpy as np

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32


class lap_runner_3(object):
	""" 
		lap_runner_3 controls a mybot to do "race" around a simple
		track. I uses two discrete pid controllers to control the
		speed and steering. It certainly is far from perfect, but 
		it can navigate a very simple track.
		Subscriptions: 	/mybot/laser/scan
		Publications: 	/cmd_vel
    	"""		

	def __init__(self):
		""" 
			The node operates from the callbacks only which are setup 
			in the init part of the code.
		""" 
		rospy.init_node('lap_runner_3')
		rospy.Subscriber('/mybot/laser/scan', LaserScan, self.scan_callback, queue_size=1)
		self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1) 
		self.twist = Twist()
		self.twist.linear.x = 0
		self.twist.angular.z = 0
		self.front_target = 2
		self.a_angle = math.pi/4
		self.lateral_target = .8
		self.time_current = rospy.get_time()
		self.delta_t = 0.1
		self.max_x = 0.1        
		self.max_theta = 0.10	
		x_p_gain = .05
		x_i_gain = 0
		x_d_gain = 0
		x_PID = self.discrete_pid_parameters(x_p_gain, x_i_gain, x_d_gain)
		self.a_x = x_PID[0]
		self.b_x = x_PID[1]
		self.c_x = x_PID[2]
		self.x_error = 0
		self.x_prev_error = 0
		self.x_pprev_error = 0
		theta_p_gain = .15
		theta_i_gain = 0
		theta_d_gain = 0.0
		theta_PID = self.discrete_pid_parameters(theta_p_gain, theta_i_gain, theta_d_gain)
		self.a_theta = theta_PID[0]
		self.b_theta = theta_PID[1]
		self.c_theta = theta_PID[2]
		self.theta_error = 0
		self.theta_prev_error = 0
		self.theta_pprev_error = 0
		self.r = rospy.Rate(1/self.delta_t)
        	rospy.spin()

	def discrete_pid_parameters(self, kp, ki, kd):	# Sets up the discrete PID controller terms 
		a = kp + ki*self.delta_t/2 + kd/self.delta_t
		b = -kp + ki*self.delta_t/2 - 2*kd/self.delta_t 
		c = kd/self.delta_t       
		return [a, b, c]

	def front_distance(self, scan):			# Scans the environment directly in front of the robot
		mid_index = len(scan.ranges) // 2
		front_dists = [val for val in scan.ranges[mid_index-2:mid_index+2]
			if not math.isnan(val)]
		return np.mean(front_dists)

	def a_distance(self, scan):			# Distance at about a 45 degree angle from directly right
		quarter_index = len(scan.ranges) // 4
		a_dists = [val for val in scan.ranges[quarter_index-2:quarter_index+2]
			if not math.isnan(val)]
		return np.mean(a_dists)

	def b_distance(self, scan):			# Distance directly to the right 
		b_dists = [val for val in scan.ranges[0:4]
			if not math.isnan(val)]
		return np.mean(b_dists)

	def x_control(self, dist):			# Controls the forward motion of robot
		prev_output = self.twist.linear.x
		x_pprev_error = self.x_prev_error
		self.x_prev_error = self.x_error
		self.x_error = -(self.front_target - dist)
		print(self.x_error)
		output = prev_output + self.a_x*self.x_error + self.b_x*self.x_prev_error + self.c_x*x_pprev_error
		if math.isnan(output):
			output = prev_output
		return output

	def theta_control(self, a_dist, b_dist):	# Controls the steering of the robot
		alpha = math.atan((a_dist*math.cos(self.a_angle) - b_dist)/(a_dist*math.sin(self.a_angle)))
		heading_error = -alpha
		AB = b_dist*math.cos(alpha)
		if math.isnan(AB):
			AB = 0
		AC = self.twist.linear.x*self.delta_t
		if math.isnan(AC):
			AC = 0
		CD = AB + AC*math.sin(alpha)
		lateral_error = self.lateral_target - CD
		prev_output = self.twist.angular.z
		theta_pprev_error = self.theta_prev_error
		self.theta_prev_error = self.theta_error
		self.theta_error = heading_error + lateral_error
		output = prev_output + self.a_theta*self.theta_error + self.b_theta*self.theta_prev_error + self.c_theta*theta_pprev_error
		if math.isnan(output):
			output = prev_output
		return output

	def scan_callback(self, scan):
		""" 
			This code functions through the callback function publishing 
			Velocity commands as it receives scan data
		"""
		front_dist = self.front_distance(scan)
		a_dist = self.a_distance(scan)
		b_dist = self.b_distance(scan)
		if front_dist > 0 and front_dist < 3:
			# Controlling forward motion
			self.twist.linear.x =  self.x_control(front_dist)
		else:
			self.twist.linear.x = self.twist.linear.x + .005
		if self.twist.linear.x > self.max_x:
			self.twist.linear.x = self.max_x
		if self.twist.linear.x < -self.max_x:
			self.twist.linear.x = -self.max_x
		if not (math.isnan(a_dist) and math.isnan(b_dist)):
			# Controlling steering motion
			self.twist.angular.z = self.theta_control(a_dist, b_dist)
		if self.twist.angular.z > self.max_theta:
			self.twist.angular.z = self.max_theta
		if self.twist.angular.z < -self.max_theta:
			self.twist.angular.z = -self.max_theta
		self.velocity_publisher.publish(self.twist) 
		self.r.sleep()

if __name__ == "__main__":
	lap_runner_3()
