#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *

# Define function to compose the Transform matrix
def T_Matrix(alpha, a, d, q):
	T = Matrix([[cos(q)				, -sin(q)				, 0				, a],
				[sin(q)* cos(alpha)	, cos(q) * cos(alpha)	, -sin(alpha)	, -sin(alpha) * d],
				[sin(q)* sin(alpha)	, cos(q) * sin(alpha)	, cos(alpha)	, cos(alpha) * d]	,
				[0						, 0						, 0				, 1]])
	return T

# Define function to perform rotations about z in 4D
def Rot_z4(theta):
	R = Matrix([	[ cos(theta)	, -sin(theta)	, 0				, 0],
					[ sin(theta)	, cos(theta)	, 0				, 0],
					[ 0				, 0				, 1				, 0],
					[ 0				, 0				, 0 			, 1]])
	return R

# Define functionto perform rotations about y in 4D
def Rot_y4(theta):
	R = Matrix([	[ cos(theta)	, 0				, sin(theta)	, 0],
					[ 0				, 1				, 0				, 0],
					[ -sin(theta)	, 0				, cos(theta)	, 0],
					[ 0				, 0				, 0 			, 1]])
	return R

# Define function to perform rotations about z in 3D
def Rot_z3(theta):
	R = Matrix([	[ cos(theta)	, -sin(theta)	, 0],
					[ sin(theta)	, cos(theta)	, 0],
					[ 0				, 0				, 1]])
	return R

# Define functionto perform rotations about y in 3D
def Rot_y3(theta):
	R = Matrix([	[ cos(theta)	, 0				, sin(theta)],
					[ 0				, 1				, 0],
					[ -sin(theta)	, 0				, cos(theta)]])
	return R

# Define functionto perform rotations about y in 3D
def Rot_x3(theta):
	R = Matrix([	[ 1				, 0				, 0],
					[ 0				, cos(theta)	, -sin(theta)],
					[ 0				, sin(theta)	, cos(theta)]])
	return R	

class RotationTracker:
	''' Class to handle rotation tracking '''
	def __init__(self):
		self.theta_prev = [0., 0., 0., 0., 0., 0.]
		self.rot_min = [-3.23	, -0.79	, -3.67	, -6.11	, -2.18	, -6.11] # minimum joint angles
		self.rot_max = [3.23	, 1.48	, 1.13	, 6.11	, 2.18	, 6.11] # maximum joint angles

	def closest_soln(self, sol1, sol2, idx):
		# because sol is output from atan2, it will be 0 -> pi and 0 -> -pi
		if abs(self.theta_prev[idx] - sol1) <= abs(self.theta_prev[idx] - sol2):
			return sol1
		else:
			return sol2

	def update_thetas(self, t1, t2, t3, t4, t5, t6):
		self.theta_prev[0] = t1
		self.theta_prev[1] = t2
		self.theta_prev[2] = t3
		self.theta_prev[3] = t4
		self.theta_prev[4] = t5
		self.theta_prev[5] = t6

def handle_calculate_IK(req):
	# create instance of RotationTracker called arm
	arm = RotationTracker()

	rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
	if len(req.poses) < 1:
		print "No valid poses received"
		return -1
	else:
		### Your FK code here
		# Create symbols
		q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') # thetha_i
		d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8') # link offsets
		a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7') # link lengths
		alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7') #twist angle

		# Define Modified DH Transformation matrix
		DH = {alpha0: 0, 		a0: 0, 		d1: 0.75, 	q1: q1,
			 alpha1: -pi/2., 	a1: 0.35, 	d2: 0, 		q2: q2 - pi/2.,
			 alpha2: 0, 		a2: 1.25, 	d3: 0,		q3: q3,
			 alpha3: -pi/2., 	a3: -0.054, d4: 1.50,	q4: q4,
			 alpha4: pi/2., 	a4: 0, 		d5: 0,		q5: q5,
			 alpha5: -pi/2., 	a5: 0, 		d6: 0,		q6: q6,
			 alpha6: 0, 		a6: 0, 		d7: 0.303, 	q7: 0}

		
		# Create individual transformation matrices
		T0_1 = T_Matrix(alpha0, a0, d1, q1).subs(DH)
		T1_2 = T_Matrix(alpha1, a1, d2, q2).subs(DH)
		T2_3 = T_Matrix(alpha2, a2, d3, q3).subs(DH)
		T3_4 = T_Matrix(alpha3, a3, d4, q4).subs(DH)
		T4_5 = T_Matrix(alpha4, a4, d5, q5).subs(DH)
		T5_6 = T_Matrix(alpha5, a5, d6, q6).subs(DH)
		T6_EE = T_Matrix(alpha6, a6, d7, q7).subs(DH)

		# Composition of homogenous transforms
		T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE

		# Initialize service response
		joint_trajectory_list = []
		for x in xrange(0, len(req.poses)):
			# IK code starts here
			joint_trajectory_point = JointTrajectoryPoint()

			# Extract end-effector position and orientation from request
			# px,py,pz = end-effector position
			# roll, pitch, yaw = end-effector orientation
			px = req.poses[x].position.x
			py = req.poses[x].position.y
			pz = req.poses[x].position.z

			(roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
				[req.poses[x].orientation.x, req.poses[x].orientation.y,
					req.poses[x].orientation.z, req.poses[x].orientation.w])

			### Your IK code here
			# Compensate for rotation discrepancy between DH parameters and URDF
			R_corr = Rot_z3(pi) * Rot_y3(-pi/2.)
			ROT_EE = Rot_z3(yaw) * Rot_y3(pitch) * Rot_x3(roll) * R_corr

			EE = Matrix([[px], [py], [pz]])

			# EE length = d7 = 0.303
			# calculate wrist centre
			WC = EE - (0.303) * ROT_EE[:,2]
					
			# Calculate joint angles using Geometric IK method
			theta1 = atan2(WC[1], WC[0])

			# using triangle and cosine laws
			s_a = 1.501
			s_b = sqrt(pow((sqrt(WC[0] * WC[0] + WC[1] * WC[1]) -0.35),2) + pow((WC[2] - 0.75), 2))
			s_c = 1.25

			# implement cosine rule for angles with known sides
			ang_a = acos((s_b * s_b + s_c * s_c - s_a * s_a) / (2 * s_b * s_c))
			ang_b = acos((s_a * s_a + s_c * s_c - s_b * s_b) / (2 * s_a * s_c))
			ang_c = acos((s_a * s_a + s_b * s_b - s_c * s_c) / (2 * s_a * s_b))

			theta2 = pi/2. - ang_a - atan2(WC[2] - 0.75, sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35)
			theta3 = pi/2. - ang_b - 0.036

			R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]

			R0_3 = R0_3.evalf(subs={q1: theta1, q2:theta2, q3:theta3})
			R3_6 = Transpose(R0_3) * ROT_EE # replaced R3_6 = R0_3.inv("LU") * ROT_EE

			theta5 = atan2(sqrt(R3_6[0,2] * R3_6[0,2] + R3_6[2,2] * R3_6[2,2]), R3_6[1,2])
			# theta 5 has two solutions, -theta5 and +theta5 (due to the square root)
			# for simplicity, choose the closest angle (however, this does not consider whether it will exceed
			# the maximum rotation limit to get there)
			#theta5 = arm.closest_soln(theta5, -theta5, 4)

			# pick the appropriate angles for t4 & 6 dependent on 5
			if theta5 > 0:
				theta4 = atan2(R3_6[2,2], -R3_6[0,2])
				theta6 = atan2(-R3_6[1,1], R3_6[1,0])
			else:
				theta4 = atan2(-R3_6[2,2], R3_6[0,2])
				theta6 = atan2(R3_6[1,1], -R3_6[1,0])

			# update the theta values in the arm instance
			arm.update_thetas(theta1, theta2, theta3, theta4, theta5, theta6)

			# Populate response for the IK request
			# In the next line replace theta1,theta2...,theta6 by your joint angle variables
			joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
			joint_trajectory_list.append(joint_trajectory_point)

		rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
		return CalculateIKResponse(joint_trajectory_list)


def IK_server():
	# initialize node and declare calculate_ik service
	rospy.init_node('IK_server')
	s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
	print "Ready to receive an IK request"
	rospy.spin()

if __name__ == "__main__":
	IK_server()
