#!/usr/bin/python
# -*- coding: utf-8 -*-

from E160_state import *
from E160_robot import *
import math
import time


class controller:

	def __init__(self, robot, logging = True):
		self.robot = robot  # do not delete this line
		self.kp = 0.109 # k_rho Note: This must be choosen to be greater than 0 to meet the strong stability condition
		self.ka = 0.3  # k_alpha Note: This must be choosen to make (ka + (5/3)*kb - (2/pi)*kp > 0) true to meet the strong stability condition
		self.kb = -0.1 # k_beta Note: This must be choosen to be less than 0 to meet the strong stability condition
		self.logging = logging

		if(logging == True):
			self.robot.make_headers(['pos_X','posY','posZ','vix','viy','wi','vr','wr'])

		self.set_goal_points()
		
		# select the controller type
		#self.controller_type='a'
		self.controller_type='p'
		

	def set_goal_points(self):
		'''
		#Edit goal point list below, if you click a point using mouse, the points programmed
		#will be washed out
		'''
		# here is the example of destination code
		
		#self.robot.state_des.add_destination(x=0,y=150,theta=3.1)    #goal point 1 (x=100,y=20,theta=0) theta must be defined in (-pi , pi) ##### commented out according to webpage
		#self.robot.state_des.add_destination(x=150,y=-100,theta=-3.1) #goal point 2 ##### commented out according to webpage
		#self.robot.state_des.add_destination(x=0,y=-250,theta=-.785/2) ##### commented out according to webpage
		#self.robot.state_des.add_destination(x=-150,y=-100,theta=.785/2)#goal point 3 ##### commented out according to webpage

	def get_robot_state(self):
		(c_posX, c_posY, c_theta) = self.robot.state.get_pos_state()  # get current position configuration
		(c_vix, c_viy, c_wi) = self.robot.state.get_global_vel_state() #get current velocity configuration, in the global frame
		(c_v, c_w) = self.robot.state.get_local_vel_state() #get current local velocity configuration
		return c_posX, c_posY, c_theta, c_vix, c_viy, c_wi, c_v, c_w
	
	# ---------- helper functions ----------
	def wrap_to_pi(self, angle):
		"""Wrap angle to (-pi, pi]."""
		while angle <= -math.pi:
			angle += 2*math.pi
		while angle > math.pi:
			angle -= 2*math.pi
		return angle

	def check_gain_convergence(self):
		"""Return True if gains satisfy convergence condition."""
		val = self.ka + (5.0/3.0)*self.kb - (2.0/math.pi)*self.kp
		return val > 0

	def track_point(self):
		'''
		Main controller method for tracking point
		'''
		
		## The following demonstrate how to get the state of the robot

		# All d_ means destination in global frame
		(d_posX, d_posY, d_theta) = self.robot.state_des.get_des_state()  #get next destination configuration

		# All c_ means current_
		c_posX, c_posY, c_theta, c_vix, c_viy, c_wi, c_v, c_w = self.get_robot_state() #get current robot state
                
		## Most of your program should be here, compute rho, alpha and beta using d_pos and c_pos

		if(self.controller_type=='a'):
			#if it is "a controller", ensure the wheel speed is not violated, you need to
			#1. turn your robot to face the target position
			#2. move your robot forward to the target position
			#3. turn your robot to face the target orientation
########################################%%%%%LG

			#1. turn your robot to face the target position
			import numpy as np
			V_R = np.array([[math.cos(c_theta)],[math.sin(c_theta)]]) #directional unit vector of the robot
			T_0__R = np.array([[math.cos(c_theta), -1*math.sin(c_theta), 0, c_posX],[math.sin(c_theta), math.cos(c_theta), 0, c_posY],[0, 0, 1, 0],[0, 0, 0, 1]]) # Transformation matrix from Origin to the robot
			T_0__R_minus = np.linalg.inv(T_0__R) # inverse of the above transformation matrix
			P_0 = np.array([[d_posX], [d_posY], [0], [1]]) # position of the point relative to the origin
			P_R = T_0__R_minus @ P_0 #Array giving the position of the point relative to robot
			divi = math.sqrt(P_R[0]**2 + P_R[1]**2)
			P_R_2 = np.array([[P_R[0]/divi],[P_R[1]/divi]]) #unit Array giving the position of the point relative to robot but only the x & y
			dot =  1 * P_R_2[0] + 0 * P_R_2[1] #V_R[0] * P_R_2[0] + V_R[1] * P_R_2[1]
			C_mag = math.sqrt(1**2 + 0**2) #||U||
			P_mag = math.sqrt(P_R_2[0]**2 + P_R_2[1]**2) #||V||
			alpha = math.acos(dot/(C_mag*P_mag))
			if 0 <= alpha <= .02:# tol 
				c_w = 0
				c_v = 10 
			else:
				c_w = .15
				c_v = 0

			left_wheel_speed = 0
			right_wheel_speed = 0
		
			print(c_theta)
		  
			#c_v = 0 #randomly assigned c_v and c_w for demonstration purpose, remove for your lab!
			#c_w = 0 #randomly assigned c_v and c_w for demonstration purpose, remove for your lab!
########################################%%%%%LG		
		elif(self.controller_type=='p'):
			##### P Controller START #####
			
			# Global deltas
			dx = d_posX - c_posX
			dy = d_posY - c_posY

			# rho: distance to the goal
			rho = math.sqrt(dx*dx + dy*dy)

			# lambda: angle from robot position to the goal (global frame)
			lambda_to_goal = math.atan2(dy, dx)

			# alpha: angle from robot heading to the goal
			alpha = self.wrap_to_pi(lambda_to_goal - c_theta)

			# beta: goal orientation error
			beta = self.wrap_to_pi(d_theta - lambda_to_goal)

			# check convergence
			if not self.check_gain_convergence():
				print("WARNING: gains may not satisfy convergence criteria: kp, ka, kb =",
					  self.kp, self.ka, self.kb)

			# Control law
			c_v = self.kp * rho
			c_w = self.ka * alpha + self.kb * beta

			# Robot parameters
			R = 12.0   # distance from center to wheel (cm)
			r = 3.0    # wheel radius (cm)

			# Convert to wheel speeds
			left_wheel_speed = (c_v - c_w * R) / r
			right_wheel_speed = (c_v + c_w * R) / r

			##### P Controller END #####
		else:
			print("no controller type is provided")
			c_v = 0
			c_w = 0

			left_wheel_speed = 0
			right_wheel_speed = 0


		# self.robot.set_motor_control(linear velocity (cm), angular velocity (rad))
		self.robot.set_motor_control(c_v, c_w)  # use this command to set robot's speed in local frame
		
		# you need to write code to find the wheel speed for your c_v, and c_w, the program won't calculate it for you.
		self.robot.send_wheel_speed(phi_l = left_wheel_speed, phi_r = right_wheel_speed) #unit rad/s

		# use the following to log the variables, use [] to bracket all variables you want to store
		# stored values are in log folder
		if self.logging == True:
			self.robot.log_data([c_posX,c_posY,c_theta,c_vix,c_viy,c_wi,c_v,c_w])


		if abs(c_posX - d_posX)< 10 and abs(math.sin(c_theta) - math.sin(d_theta)) < 0.25 and abs(c_posY - d_posY)< 10 and abs(math.cos(c_theta) - math.cos(d_theta)) < 0.25: #you need to modify the reach way point criteria
			if(self.robot.state_des.reach_destination()): 
				print("final goal reached")
				self.robot.set_motor_control(.0, .0)  # stop the motor
				return True
			else:
				print("one goal point reached, continute to next goal point")
				

		return False #just return False, don't remove it.
