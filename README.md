## Project: Kinematics Pick & Place

---

[//]: # (Image References)

[image1]: DH_Sketch.png
[image2]: IK.png
[image3]: IK2.png
[image4]: IK3.png
[image5]: grasp.png
[image6]: release.png

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

![DH2][image3]

3.1 Theta 1
```py
theta1 = (atan2(wy, wx)).evalf()
```

3.2 Theta 2 and 3

The [law of cosines](https://en.wikipedia.org/wiki/Law_of_cosines) was used calculate theta 2 and 3.

```py
s1 = sqrt(wx**2 + wy**2) - a_1
s2 = wz - d_1
s3 = sqrt(s2**2 + s1**2)
s4 = sqrt(a_3**2 + d_4**2)
beta1 = atan2(s2, s1)

D2 = (a_2**2 + s3**2 - s4**2) / (2 * a_2 * s3)
beta2 = atan2(sqrt(1 - D2**2), D2)

D3 = (a_2**2 + s4**2 - s3**2) / (2 * a_2 * s4)
beta3 = atan2(sqrt(1 - D3**2), D3)

beta4 = atan2(-a_3, d_4)
```

```py
theta2 = ((pi / 2) - beta2 - beta1).evalf()
theta3 = ((pi / 2) - beta4 - beta3).evalf()
```

![DH3][image4]

3.3 Theta 4, 5 and 6

The orientation matrix of R0\_3 is extracted from T0\_3 and its inverse is found. When R0\_6 is multiplied by the inverse of R0\_3, the result is R3\_6.

```py
R0_3 = T0_3[0:3, 0:3]
R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
R0_3_inv = R0_3.transpose()
R3_6 = R0_3_inv * R0_6
```

```py
r13 = R3_6[0, 2]
r33 = R3_6[2, 2]
r23 = R3_6[1, 2]
r21 = R3_6[1, 0]
r22 = R3_6[1, 1]
r12 = R3_6[0, 1]
r32 = R3_6[2, 1]

theta5 = (atan2(sqrt(r13**2 + r33**2), r23)).evalf()
theta4 = (atan2(r33, -r13)).evalf()
theta6 = (atan2(-r22, r21)).evalf()
```

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results.


1. The first obstacle is that the debugging process of the IK process could be labor-some and tedious. It is solved by heavily utilized the script `IK_debug.py` to speed up the trial-and-error process. Adding the FK calculation in the script helps a lot. Since an arm can have multiple orientations, the errors can be non zero. But by including FK calculations, we can still see if the correct EE is reached.

2. The second obstacle is that the calculation of IK could be slow. It is solved by moving the `sympy` math out of the `for` loop. Generating transformation matrices is a time consuming process, and since this process needs to happen only once, it is removed from the loop. As a result, the run time is reduced significantly.

The screen-shots below show the grasping process of the robot arm as well as the bin falling into bin after releasing.
![grasp][image5]
![release][image6]
