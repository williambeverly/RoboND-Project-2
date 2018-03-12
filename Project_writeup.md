## Project: Kinematics Pick & Place

**Introduction:**  

This write-up addresses the individual rubric points, and describes how each point was addressed in my implementation. All line numbers are in reference to IK_server.py.

A Jupyter Notebook and PDF have been included in the repository as supporting documentation. The aim of the project is to pick up on object from a shelf, and place this object into a bin, using a robotic arm. This project involved the composition and analysis of Forward Kinematics (FK) and Inverse Kinematic (IK) functions for the Kuka KR210 robotic arm. This write-up documents the following items:
* Annoted figure of link assignments and joint rotations
* DH parameter table
* Annotated figure for theta1, theta2 and theta3 determinations
* Extracted code and results for FK and IK debugging
* Images for project implementation
* **IK_server.py included in repository**

[//]: # (Image References)

[image1]: ./imgs/image1.png
[image2]: ./imgs/transform_function.png
[image3]: ./imgs/ht_composition.png
[image4]: ./imgs/image2.png
[image5]: ./imgs/r3_6.png
[image6]: ./imgs/r3_6_calcs.png
[image7]: ./imgs/testcase_1.png
[image8]: ./imgs/display_path.png
[image9]: ./imgs/complete.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.

Welcome to my write-up!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

The forward_kinematics demo was run and the kr210.urdf.xacro file was evaluated to obtain the specific lengths for the links. The figure below shows the link assignments and joint rotations. For reference, the parameters are assigned as follows:

* twist angle = alpha(i-1) = z(i-1) to z(i), about x(i-1)
* link length = a(i-1) = z(i-1) to z(i), about x(i-1)
* link offset = d(i) = x(i-1) to x(i), about z(i)
* joint angle = theta(i) = x(i-1) to x(i), about z(i)

![alt text][image1]

The resulting DH parameter table is as follows:

| Links | alpha(i-1) |   a(i-1)   |    d(i)    |  theta(i)  |
|-------|------------|------------|------------|------------|
| 0 -> 1 |     0      |     0      |     0.75   |     0      |
| 1 -> 2 |  -pi/2     |   0.35     |     0      |  theta(i)-pi/2|
| 2 -> 3 |     0      |   1.25     |     0      |     0      |
| 3 -> 4 |  -pi/2     |  -0.054    |     1.50   |     0      |
| 4 -> 5 |   pi/2     |     0      |     0      |     0      |
| 5 -> 6 |  -pi/2     |     0      |     0      |     0      |
|6 -> EE|     0      |     0      |     0.303  |     0      |


#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

A function was created (lines 21-27) individual transform matrices, as shown in the figure below:

![alt text][image2]

The function was utilised to generate a generalized homogenous transform between the base_link and gripper_link, as shown in the figure below:

![alt text][image3]

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

The figure below shows the angle assignments and relevent calculations for theta1, theta2 and theta3. The cosine rule for known sides can be applied to calculate each angle. The values from the DH table are substituted in the relevent places.

![alt text][image4]

Therefore, solving for theta1, 2 and 3 via the geometric method, and utilising the EE position (with correction applied due to the difference in URDF and Gazebo frames) and the homogenous transforms up the wrist centre, the following formula can be applied to calculate R3_6.

![alt text][image5]

**Theta4**, **theta5** and **theta6** are extracted from the R3_6 matrix. The calculated R3_6 matrix is shown in the figure below:

![alt text][image6]

The implementation of the code located at lines 155 - 187. Additionally, the implementation was tested on three test cases for IK_debug.py. The results demonstrate that the EE position error is very low. In test cases 2 and 3, there are errors in the theta values, as the joint angle solution is not unique, so multiple different joint angles can yield the same EE position. A snapshot of the performance shown in the figure below:

![alt text][image7]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

The IK_debug code was transferred to the IK_server.py code. The demo mode flag in inverse_kinematics.launch was set to false. The project was launched by sourcing the workspace and running:
* `cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts`
* `./safe_spawner.sh`
In a new terminal, after workspace sourcing the IK_server.py was launched by:
* `rosrun kuka_arm IK_server.py`
The first image is displaying the calculated path and the second image is the completed Pick and Place operation.

![alt text][image8]
![alt text][image9]

The implementation was successfully able to pick up the object from the shelf, and release the object into the bin, meeting the minimum requires of 8/10 successful runs. As an observation, some motions resulted in the joints rotating unnecessarily - for example the end effector rotating by 180 degrees along joint 6. For future work, I would like to minimise the rotations, and give the arm the intelligence to select the best joint angle, based on the previously stored angle.
