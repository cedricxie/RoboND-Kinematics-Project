## Project: Kinematics Pick & Place

---

[//]: # (Image References)

[image1]: DH_Sketch.PNG
[image2]: IK.png
[image3]: IK2.png
[image4]: IK3.png

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

The figure below shows the coordinate systems for each joint following DH conventions.

![DH Sketch][image1]

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | - pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | q3
3->4 | - pi/2 | -0.054 | 1.5 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | - pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | 0


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

Owing to the fact that last three neighboring joint axes intersect at a single point (**wrist center**), this problem can be **kinematically decouples** into the calculation of position and orientation of the end effector respectively.  

Special thanks to Wenjin Tao for the images he shared to calculate thetas 1, 2, and 3.

![DH][image2]

##### Inverse position calculation

![DH2][image3]

##### Inverse orientation calculation

![DH3][image4]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results.


1. Heavily utilized the script `IK_debug.py` to speed up the trial-and-error process. Adding the FK calculation in the script helps a lot. Since an arm can have multiple orientations, the errors can be non zero. But by including FK calculations, we can still see if the correct EE is reached.

2. Moved the `sympy` math out of the `for` loop. Generating transformation matrices is a time consuming process, and since this process needs to happen only once, it is removed from the loop. As a result, the run time is reduced significantly.
