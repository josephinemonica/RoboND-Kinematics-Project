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


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### Your FK code here
        # Create symbols
	q1, q2, q3, q4, q5, q6, q7 =symbols('q1 q2 q3 q4 q5 q6 q7')
    a0, a1, a2, a3, a4, a5, a6 =symbols('a0 a1 a2 a3 a4 a5 a6')
    d1, d2, d3, d4, d5, d6, d7=symbols('d1 d2 d3 d4 d5 d6 d7')
    alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6=symbols('alpha0 alpha1 alpha2 alpha3 alpha4 alpha5 alpha6')
	# Create Modified DH parameters
    DH_param={alpha0:0, alpha1:-pi/2, alpha2:0, alpha3:-pi/2, alpha4:pi/2, alpha5:-pi/2, alpha6:0,
    a0:0, a1:0.35, a2:1.25, a3:-0.054, a4:0, a5:0, a6:0,
    d1:0.75, d2:0, d3:0, d4:1.5, d5:0, d6:0, d7:0.303,
    q1:q1, q2:q2-pi/2, q3:q3, q4:q4, q5:q5, q6:q6, q7:0}
    # Define Modified DH Transformation matrix
    def T(theta, a, d, alpha): 
        return Matrix([[cos(theta), -sin(theta), 0, a],
	            [sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d],
	            [sin(theta)*sin(alpha), cos(theta)* sin(alpha), cos(alpha), cos(alpha)*d],
	            [0, 0, 0, 1]])
    # Create individual transformation matrices
    T0_1=T(q1,a0,d1,alpha0).subs(DH_param)
    T1_2=T(q2,a1,d2,alpha1).subs(DH_param)
    T2_3=T(q3,a2,d3,alpha2).subs(DH_param)
    T3_4=T(q4,a3,d4,alpha3).subs(DH_param)
    T4_5=T(q5,a4,d5,alpha4).subs(DH_param)
    T5_6=T(q6,a5,d6,alpha5).subs(DH_param)
    T6_7=T(q7,a6,d7,alpha6).subs(DH_param)
	# Extract rotation matrices from the transformation matrices
    R0_1=T0_1[0:3,0:3]
    R1_2=T1_2[0:3,0:3]
    R2_3=T2_3[0:3,0:3]
    R3_4=T3_4[0:3,0:3]
    R4_5=T4_5[0:3,0:3]
    R5_6=T5_6[0:3,0:3]
    R6_7=T6_7[0:3,0:3]

    T0_7=T0_1*T1_2*T2_3*T3_4*T4_5*T5_6*T6_7
    R0_7=T0_7[0:3,0:3]
        ###

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
            #elementary rotation matrices:
        def Qx(t):
            return Matrix([[1,0,0],
		                    [0,cos(t),-sin(t)],
		                    [0,sin(t),cos(t)]])
        def Qy(t):
            return Matrix([[cos(t),0,sin(t)],
                    		[0,1,0],
                    		[-sin(t),0,cos(t)]])
        def Qz(t):
            return Matrix([[cos(t),-sin(t),0],
		                    [sin(t),cos(t),0],
		                    [0,0,1]])
        # Compensate for rotation discrepancy between DH parameters and Gazebo
        R_EE=Qz(yaw)*Qy(pitch)*Qx(roll)
        R_compensate=Qz(pi)*Qy(-pi/2) #see KR210 Forward Kinematics 2 video
        R_EE=R_EE*R_compensate
        # Calculate joint angles using Geometric IK method
        #Positioning problem-> find q1,q2,q3
        #length from wrist center to EE
        #wrist center location
        r_wc=Matrix([[px],[py],[pz]])-0.303*R_EE*Matrix([[0],[0],[1]])
        #
        #q1
        theta1=atan2(r_wc[1],r_wc[0])

        #q2 and q3
        A=1.501
        B=sqrt( (sqrt(r_wc[0]*r_wc[0]+r_wc[1]*r_wc[1])-0.35)**2 + (r_wc[2]-0.75)**2 )
        C=1.25

        a=acos((B*B+C*C-A*A)/(2*B*C))
        b=acos((A*A+C*C-B*B)/(2*A*C))
        c=acos((A*A+B*B-C*C)/(2*A*B))

        theta2=(pi/2-a-atan2(r_wc[2]-0.75,sqrt(r_wc[0]*r_wc[0]+r_wc[1]*r_wc[1])-0.35) ).evalf()
        theta3=(pi/2-b-0.036).evalf()

        #Orientation problem -> find q4,q5,q6
        #using q1 q2 and q3, calculate R0_3
        R0_3=(R0_1*R1_2*R2_3)
        R0_3=R0_3.subs([(q1,theta1), (q2,theta2), (q3,theta3)])
        R3_6=R0_3**-1*R_EE
        theta4=atan2(R3_6[2,2],-R3_6[0,2])
        theta5=atan2(sqrt(R3_6[0,2]*R3_6[0,2]+R3_6[2,2]*R3_6[2,2]),R3_6[1,2])
        theta6=atan2(-R3_6[1,1],R3_6[1,0])
        ###

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
