#Goal: Develop a control system for the youBot Mobile Manipulator
#that can perform pick and place tasks in a dynamic environment.
#The system will integrate odometry, feedback control, and manipulation planning.

#Input:
#1. Initial cube configuration
#2. Desired cube configuration
#3. Robot's initial position and orientation
#4. T_se of the end-effector
#5. Gains for feedback control

#Output:
#1. csv file with robot & end effector trajectory
    - with these values: [chassis phi, chassis x, chassis y, J1, J2, J3, J4, J5, W1, W2, W3, W4, gripper state]
    where J1 to J5 are the arm joint angles and W1 to W4 are the four wheel angles. 
#2. 6-vector of end-effector error

