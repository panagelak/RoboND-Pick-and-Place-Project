## Project: Kinematics Pick & Place

**Steps to complete the project:**  

1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: http://i67.tinypic.com/2pzb0y0.png
[image2]: http://i66.tinypic.com/2nib3g0.png
[image3]: http://i63.tinypic.com/2ur5t6u.jpg
[image4]: http://i68.tinypic.com/rbk2u1.jpg
[image5]: http://i64.tinypic.com/2j4va50.jpg
[image6]: http://i67.tinypic.com/1cdqx.jpg
[image7]: http://i64.tinypic.com/34pyo46.jpg
## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---


### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

![alt text][image1]
![alt text][image2]

**After examining the kr210.urdf.xacro file used for describing the robot model, kinodynamic properties, visual elements and even model sensors, we go to the section joints and we derive the translation bewteen reference frames (since we have no rotation) therefore we can understand the dimensions of the robot, and then we can derive our DH paramaters**

![alt text][image3]

* **Z axis direction is the direction of the joint axes**
* **X axis is usually common normal of z axis**
* **Practice is needed to place these axes so you can minimize as many DH paramaters as possible (x0 is placed in the base)**
* **alpha = (Twist angle ) , a (link length), d (link offset), q (joint variable revolute)**
* **alpha0 = 0 : no angle bewteen Zi-1 and Zi measured along Xi-1 , a0 = 0: distance bewteen Zi-1 to Zi measured along Xi-1**
* **d1 = 0.33 + 0.42 (from urdf) =0.75 : signed distance Xi-1 TO Xi measured along Zi**
* **alpha1 = -pi/2 : (there is rotation bewteen Z1 and Z2), a1 = 0.35 (distance bewteen Z1 and Z2 along X1)**
* **q2 = q2 -pi/2 : (90 degree difference bewteen X1 and X2 around Z2 axis)**
* **a2 = 1.25 , alpha3 = -pi/2 (measured along x3), a3 = -0.054 (distance bewteen Z3 and Z4 measured along X3)**
* **d3 = 1.5 (from urdf) , alpha4 = pi/2, alpha5 = -pi/2, d6 = 0.303 (distance of EE from WC)** 
* **In O5 we have the Wrist Center of the last 3 joints since they are all revolute (X4,X5,X6)**

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | - pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | q3
3->4 |  -pi/2 | -0.054 | 1.5 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | -pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | 0

![alt text][image4]

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's where you can draw out and show your math for the derivation of your theta angles. 

![alt text][image6]

* **Choosing among the two solutions for theta5**

![alt text][image7]
### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.

* **First we construct the DH table and after that we build the individual transformation matrices aff all joints based on this table.**
* **After, we post multiply these matrices to construct the homogeneous transform of the base link to the End Effector while,
also extracting the rotation matrices from them.**
* **Then we calculate Rrpy based of the position of EE which is given while also correcting for rotation difference bewteen the urdf and DH Table.**
* **Finally we calculate the inverse kinematics and find all the theta angles for each pose given to us in order to complete the trajectory given to us by MoveIt.(We choose the optimal solution based on the theta5 value in order to avoid redudant rotations).**
* **The implementation might fail because of gimbal lock (we would need a way to avoid these or use quatertions instead of euler angle in order to avoid reference singularities altogether)**
* **Also the MoveIt doesnt always give us optimal trajectories.**
* **The code could be implemented to run faster using different data structures or loading all of the pre calculations needed, from pickle files**

And just for fun, another example image:
![alt text][image5]


