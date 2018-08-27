## Project: Kinematics Pick & Place

**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png

[imageT]: ./misc_images/T.png
[imageT0_1]: ./misc_images/T0_1.png
[imageT1_2]: ./misc_images/T1_2.png
[imageT2_3]: ./misc_images/T2_3.png
[imageT3_4]: ./misc_images/T3_4.png
[imageT4_5]: ./misc_images/T4_5.png
[imageT5_6]: ./misc_images/T5_6.png
[imageT6_7]: ./misc_images/T6_7.png

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

Links | alpha(i-1)  | a(i-1)  | d(i-1)  | theta(i)
---   | ---         | ---     | ---     | ---
0->1  | 0           | 0       | 0.75    | q1
1->2  | - pi/2      | 0.35    | 0       | -pi/2 + q2
2->3  | 0           | 1.25    | 0       | q3
3->4  |  -pi/2      | -0.054  | 1.5     | q4
4->5  | pi/2        | 0       | 0       | q5
5->6  | -pi/2       | 0       | 0       | q6
6->EE | 0           | 0       | 0.303   | 0

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

The general form of the transformation matrix, based on the DH convention, is:

![Transformation Matrix T][imageT]

Using the above formula, we derive the individual transformation matrices about each joint:
![Transformation Matrix T0_1][imageT0_1] <br />
![Transformation Matrix T1_2][imageT1_2] <br />
![Transformation Matrix T2_3][imageT2_3]<br />
![Transformation Matrix T3_4][imageT3_4]<br />
![Transformation Matrix T4_5][imageT4_5]<br />
![Transformation Matrix T5_6][imageT5_6]<br />
![Transformation Matrix T6_7][imageT6_7]<br />

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

Since the arm has a spherical wrist, the inverse kinematics problem can be decoupled into positioning and orientation problem.

##### Positioning (Involving &theta;&#8321;, &theta;&#8322;, and &theta;&#8323;)
* Find the location of the wrist center (wc) relative to the base frame.
* Find joint angles (&theta;&#8321;, &theta;&#8322;, abd &theta;&#8323;) that result to such wrist center position

![alt text][image2]

##### Orientation (Involving &theta;&#8324;, &theta;&#8325;, and &theta;&#8326;)
* Using &theta;&#8321;, &theta;&#8322;, and &theta;&#8323;, compute the rotation matrix R0_3
* Given the desired orientation of the EE, which can be expressed in terms of its rotation matrix R0_7, we can compute R3_6
* By comparing the known formula of R3_6 and the computed R3_6 above, we can get the joint angles &theta;&#8324;, &theta;&#8325;, and &theta;&#8326; that result into the desired orientation.



### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]


