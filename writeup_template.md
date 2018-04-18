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
### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

The goal of forward kinematics is to calculate the pose of the end-effector given all the joint angles, six in this case. You should use the modified Denavit-Hartenberg parameter convention to associate reference frames to each link and complete the DH parameter table.

Using your DH parameter table, you can create individual transforms between various links. It is best to create the transforms symbolically, and then substitute numerical values for the non-zero terms as the last step.

Inverse kinematics (IK) is essentially the opposite idea of forwards kinematics. In this case, the pose (i.e., position and orientation) of the end effector is known and the goal is to calculate the joint angles of the manipulator.

![alt text][image1]

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


![][image2]    

The DH convention uses four individual transforms,

![][image8]
![][image5]  

to describe the relative translation and orientation of link (i-1) to link (i). In matrix form, this transform is,

![][image9]  

In addition to individual transforms, you can also determine a complete homogeneous transform between the base_link and the gripper_link (end-effector) using just the end-effector pose (position+rotation). Remember that the homogeneous transform consists of a rotation part and a translation part as follows:
 
![][image10]  

where Px, Py, Pz represent the position of end-effector w.r.t. base_link and RT represent the rotation part. Keep in mind that RT can be constructed using the Roll-Pitch-Yaw angles of the end-effector (which will be provided from the simulator).




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


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:  

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/MdpdmbwKlCA/0.jpg)](https://www.youtube.com/watch?v=MdpdmbwKlCA)


