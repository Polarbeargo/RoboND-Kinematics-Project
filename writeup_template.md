## Project: Kinematics Pick & Place

---
![][image6]  

* This project is working with a simulation of Kuka KR210 to pick up cans from a shelf and then put them in a dropbox.  
  
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
[image4]: ./misc_images/misc4.png
[image5]: ./misc_images/misc5.png
[image6]: ./misc_images/misc6.png
[image7]: ./misc_images/misc7.png
[image8]: ./misc_images/misc8.png
[image9]: ./misc_images/misc9.png
[image10]: ./misc_images/misc10.png
[image11]: ./misc_images/image-3.png
[image12]: ./misc_images/image-4.png
[image13]: ./misc_images/image-5.png
[image14]: ./misc_images/misc11.png
[image15]: ./misc_images/misc12.png
### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

The goal of forward kinematics is to calculate the pose of the end-effector given all the joint angles, six in this case. By Using the modified Denavit-Hartenberg parameter convention to associate reference frames to each link and complete the DH parameter table.

Using DH parameter table, Create individual transforms between various links. It is best to create the transforms symbolically, and then substitute numerical values for the non-zero terms as the last step.

Inverse kinematics (IK) is essentially the opposite idea of forwards kinematics. In this case, the pose (i.e., position and orientation) of the end effector is known and the goal is to calculate the joint angles of the manipulator.

![][image1]  

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.
  
![][image7]  

* DH parameters used specifically in this project as follow:

|ID   |![alpha][alpha_i-1] |![a][a] |![d][d] |![theta][theta]    |
|:---:|:------------------:|:------:|:------:|:-----------------:| 
|    1|                  0 |      0 |   0.75 |     ![q1][theta1] |
|    2|      ![-pi2][-pi2] |   0.35 |      0 |  ![q2][theta2-90] |
|    3|                  0 |   1.25 |      0 |     ![q3][theta3] |
|    4|      ![-pi2][-pi2] | -0.054 |   1.50 |     ![q4][theta4] |
|    5|        ![pi2][pi2] |      0 |      0 |     ![q5][theta5] |
|    6|      ![-pi2][-pi2] |      0 |      0 |     ![q6][theta6] |
|   EE|                  0 |      0 |  0.303 |                 0 |

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.  

Since the last three joints in our robot are revolute and their joint axes intersect at a single point, we have a case of spherical wrist with joint_5 being the common intersection point and hence the wrist center.

This allows us to kinematically decouple the IK problem into Inverse Position and Inverse Orientation
Since we have the case of a spherical wrist involving joints 4,5,6, the position of the wrist center is governed by the first three joints. We can obtain the position of the wrist center by using the complete transformation matrix we derived in the last section based on the end-effector pose.

For the sake of simplification, let us symbolically define our homogeneous transform as following:  

![][image14]  

where l, m and n are orthonormal vectors representing the end-effector orientation along X, Y, Z axes of the local coordinate frame.

Since n is the vector along the z-axis of the gripper_link, we can say the following:  

![][image15]  

Where,
Px, Py, Pz = end-effector positions

Wx, Wy, Wz = wrist positions

d6 = from DH table

l = end-effector length

Now, in order to calculate nx, ny, and nz, let's continue from the previous section where you calculated the rotation matrix to correct the difference between the URDF and the DH reference frames for the end-effector.

After obtain this correctional rotation matrix,  next calculate end-effector pose with respect to the base_link. In the Compositions of Rotations section, we covered different conventions regarding Euler angles, and how to choose the correct convention.

One such convention is the x-y-z extrinsic rotations. The resulting rotational matrix using this convention to transform from one fixed frame to another, would be:
```
Rrpy = Rot(Z, yaw) * Rot(Y, pitch) * Rot(X, roll) * R_corr
```
Where R_corr is the correctional rotation matrix.

The roll, pitch, and yaw values for the gripper are obtained from the simulation in ROS. But since these values are returned in quaternions, you can use the transformations.py module from the TF package. The euler_from_quaternions() method will output the roll, pitch, and yaw values.
Now that with the wrist center position, follow the simple math presented in this section to derive the equations for first three joints.

Calculating theta 1 will be relatively straightforward once you have the wrist center position from above. Theta 2 and theta 3 can be relatively tricky to visualize. The following diagram depicts the angles for you.
![][image2]  
The labels 2, 3 and WC are Joint 2, Joint 3, and the Wrist Center, respectively. You can obtain, or rather visualize, the triangle between the three if you project the joints onto the z-y plane corresponding to the world reference frame. From your DH parameters, you can calculate the distance between each joint above. Using trigonometry, specifically the Cosine Laws, you can calculate theta 2 and theta 3.

For the Inverse Orientation problem, we need to find values of the final three joint variables.

Using the individual DH transforms we can obtain the resultant transform and hence resultant rotation by:

```
R0_6 = R0_1*R1_2*R2_3*R3_4*R4_5*R5_6
```

Since the overall RPY (Roll Pitch Yaw) rotation between base_link and gripper_link must be equal to the product of individual rotations between respective links, following holds true:
```
R0_6 = Rrpy
```
where,

Rrpy = Homogeneous RPY rotation between base_link and gripper_link as calculated above.

We can substitute the values we calculated for joints 1 to 3 in their respective individual rotation matrices and pre-multiply both sides of the above equation by inv(R0_3) which leads to:

R3_6 = inv(R0_3) * Rrpy
The resultant matrix on the RHS (Right Hand Side of the equation) does not have any variables after substituting the joint angle values, and hence comparing LHS (Left Hand Side of the equation) with RHS will result in equations for joint 4, 5, and 6.  

to describe the relative translation and orientation of link (i-1) to link (i). In matrix form, this transform is,

![][image9]    
Thus, for a manipulator with n-joints the overall transformation between the base and end effector involves n-multiples of Equation (1). Clearly this can result in complicated, highly non-linear equations that, in general, can have zero or multiple solutions. Further, these mathematical solutions may in fact violate real-world joint limits so care is needed when choosing among the mathematically possible solution(s). Let???s pause for a moment and think about what these different solutions physically mean.

Consider the case of an anthropomorphic (RRR) manipulator. The name comes from the fact that joint 1 is analogous to the human hip joint (imagine twisting at the waist about a vertical axis), joint 2 is the shoulder, and joint 3 is the elbow. If all we care about is the position of the end effector, there are four possible ways to reach a 3D point in the manipulator's workspace. Notice the shading on each revolute joint. The top row corresponds to the elbow up solutions. Column 1 corresponds solutions with hip joint = theta versus column 2 with hip joint = theta + pi.  
![][image5]   

![][image4]  

Step 1: is to complete the DH parameter table for the manipulator. Hint: place the origin of frames 4, 5, and 6 coincident with the WC.

Step 2: is to find the location of the WC relative to the base frame. Recall that the overall homogeneous transform between the base and end effector has the form, 

![][image11]  

If, for example, you choose z4 parallel to z6 and pointing from the WC to the EE, then this displacement is a simple translation along z6. The magnitude of this displacement, let???s call it d, would depend on the dimensions of the manipulator and are defined in the URDF file. Further, since r13, r23, and r33 define the Z-axis of the EE relative to the base frame, the Cartesian coordinates of the WC is,  

![][image12]  

Step 3: find joint variables, q1, q2 and q3, such that the WC has coordinates equal to equation (3). This is the hard step. One way to attack the problem is by repeatedly projecting links onto planes and using trigonometry to solve for joint angles. Unfortunately, there is no generic recipe that works for all manipulators so you will have to experiment. The example in the next section will give you some useful guidance.

Step 4: once the first three joint variables are known,R via application of homogeneous transforms up to the WC.

Step 5: find a set of Euler angles corresponding to the rotation matrix,  

![][image13]  

Step 6: choose the correct solution among the set of possible solutions

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

Check the source code and read annotation for detailed explanation.

And just for fun, another demo video:  

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/MdpdmbwKlCA/0.jpg)](https://www.youtube.com/watch?v=MdpdmbwKlCA)





