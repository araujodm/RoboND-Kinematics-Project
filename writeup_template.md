## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.

It was done by creating the /catkin_ws. As mentioned during the course, catkin is the official build system of ROS.

2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  

https://github.com/araujodm/RoboND-Kinematics-Project/blob/patch-1/misc_images/ROS%20Workspace.png


3. Experiment with the forward_kinematics environment and get familiar with the robot.

https://github.com/araujodm/RoboND-Kinematics-Project/blob/patch-1/misc_images/Forward%20Kinematics.png


4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).

https://github.com/araujodm/RoboND-Kinematics-Project/blob/patch-1/misc_images/Kuka%20arm%20demo%20RViz.png
https://github.com/araujodm/RoboND-Kinematics-Project/blob/patch-1/misc_images/kuka%20arm%20demo%20Gazebo.png

5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).

Check out DH table here:
https://github.com/araujodm/RoboND-Kinematics-Project/blob/patch-1/misc_images/DH_Table.pdf

Here a screenshot from the urdf file:
https://github.com/araujodm/RoboND-Kinematics-Project/blob/patch-1/misc_images/kuka_arm_xacro_urdf.png

T0_1 =  Matrix([[            cos(q1),            -sin(q1),            0,              a0]
                [sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1]
                [sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1]
                [                  0,                   0,            0,               1]])

T1_2 =  Matrix([[            cos(q2),            -sin(q2),            0,              a1]
                [sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2]
                [sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2]
                [                 0,                    0,            0,               1]])

T2_3 =  Matrix([[            cos(q3),            -sin(q3),            0,              a2]
                [sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3]
                [sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3]
                [                  0,                   0,            0,               1]])

T3_4 =  Matrix([[            cos(q4),            -sin(q4),            0,              a3]
                [sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4]
                [sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4]
                [                  0,                   0,            0,               1]])
                  
T4_5 =  Matrix([[            cos(q5),            -sin(q5),            0,              a4]
                [sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5]
                [sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5]
                [                  0,                   0,            0,               1]])
     
T5_6 =  Matrix([[            cos(q6),            -sin(q6),            0,              a5]
                [sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6]
                [sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6]
                [                  0,                   0,            0,               1]])
                
T6_EE =  Matrix([[           cos(q7),            -sin(q6),            0,              a6]
                [sin(q7)*cos(alpha6), cos(q7)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d7]
                [sin(q7)*sin(alpha6), cos(q7)*sin(alpha6),  cos(alpha6),  cos(alpha6)*d7]
                [                  0,                   0,            0,               1]])          

T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE


6. Fill in the `IK_server.py` with your Inverse Kinematics code. 

Here is my code:

#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
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
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') #joint angles
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8') #link offset
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7') #link length
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7') #Twist angle
	#
	#   
	### Create Modified DH parameters (DH_Table)
	s = {alpha0:     0, a0:      0, d1:      0.75, q1:          q1,
         alpha1: -pi/2, a1:   0.35, d2:         0, q2:    -pi/2+q2,
         alpha2:     0, a2:   1.25, d3:         0, q3:          q3,
         alpha3: -pi/2, a3: -0.054, d4:       1.5, q4:          q4,
         alpha4:  pi/2, a4:      0, d5:         0, q5:          q5,
         alpha5: -pi/2, a5:      0, d6:         0, q6:          q6,
         alpha6:     0, a6:      0, d7:     0.303, q6:           0,
        }
	#            
	# Define Modified DH Transformation matrix
	#

    def TF_Matrix(alpha, a, d, q)
        TF =  Matrix([[           cos(q),           -sin(q),           0,             a]
                      [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d]
                      [sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d]
                      [                0,                 0,           0,             1]])
        return TF

	#
	# Create individual transformation matrices

    T0_1 = TF_Matrix(alpha0, a0, d1, q1).subs(s)
    T1_2 = TF_Matrix(alpha1, a1, d2, q2).subs(s)
    T2_3 = TF_Matrix(alpha2, a2, d3, q3).subs(s)
    T3_4 = TF_Matrix(alpha3, a3, d4, q4).subs(s)
    T4_5 = TF_Matrix(alpha4, a4, d5, q5).subs(s)
    T5_6 = TF_Matrix(alpha5, a5, d6, q6).subs(s)
    T6_EE = TF_Matrix(alpha6, a6, d7, q7).subs(s)

    T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE
	#
	#
	# Extract rotation matrices from the transformation matrices
	#
	#

    r, p, y = symbols('r p y')

    ROT_x = Matrix([[       1,      0,       0]
                    [       0, cos(r), -sin(r)]
                    [       0, sin(r),  cos(r)]]) # ROW


    ROT_y = Matrix([[  cos(p),      0,  sin(p)]
                    [       0,      1,       0]
                    [ -sin(p),      0,  cos(p)]]) #PITCH

    ROT_z = Matrix([[  cos(y), -sin(y),      0]
                    [  sin(y),  cos(y),      0]
                    [       0,       0,      1]]) # YAW



    ROT_EE = ROT_z * ROT_y * ROT_x

    
    ROT_Error = ROT_z.subs(y,radians(180)*ROT_Y.subs(p,radians(-90)))

    ROT_EE = ROT_EE * ROT_Error

    ROT_EE =  ROT_EE.subs({'r': roll, 'p': pitch, 'y': yaw})

    EE = Matrix ([[px],
                  [py],
                  [pz]])

    WC = EE - 0.303 * ROT_EE[:,2]

    
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
	    # Compensate for rotation discrepancy between DH parameters and Gazebo
	    #
	    #
	    # Calculate joint angles using Geometric IK method
	    theta1 = atan2(WC[1], WC[0])

        #SSS triangule for theta2 and theta3

        side_a = 1.501
        side_b = sqrt(pow((sqrt(WC[0]*WC[0] + WC[1]*WC[1]) - 0.35), 2) + pow[(WC[2]-0.75), 2])
        side_c = 1.25

        angle_a = acos((side_b*side_b + side_c*side_c - side_a*side_a) / (2 * side_b * side_c))
        angle_b = acos((side_a*side_a + side_c*side_c - side_b*side_b) / (2 * side_a * side_c))
        angle_c = acos((side_a*side_a + side_b*side_b - side_c*side_c) / (2 * side_a * side_b))

        theta2 = pi/2 - angle_a - atan2(WC[2] - 0.75, sqrt(WC[0]*WC[0] + WC[1]*WC[1] - 0.35))
        theta3 = pi/2 - (angle_b + 0.036)  #0.036 accounts for sag in link 4 of -0.054

        R0_3 = T0_1[0:3, 0:3] * T1_2[0:3, 0:3] * T2_3[0:3, 0:3]
        R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
        R3_6 = R0_3.inv("LU") * ROT_EE

        # EULER ANGLES from rotation matrix

        theta4 = atan2(R3_6[2,2], -R_6[0, 2])
        theta5 = atan2(sqrt(R3_6[0, 2]*R3_6[0, 2] + R3_6[2, 2]* R3_6[2, 2]), R3_6[1.25])
        theta6 = atan2( -R3_6[1, 1], R3_6[1, 0])
	    #
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





[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Here is an example of how to include an image in your writeup.

![alt text][image1]

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | L1 | qi
1->2 | - pi/2 | L2 | 0 | -pi/2 + q2
2->3 | 0 | 0 | 0 | 0
3->4 |  0 | 0 | 0 | 0
4->5 | 0 | 0 | 0 | 0
5->6 | 0 | 0 | 0 | 0
6->EE | 0 | 0 | 0 | 0


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's where you can draw out and show your math for the derivation of your theta angles. 

![alt text][image2]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]


