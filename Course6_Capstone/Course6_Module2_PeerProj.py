# %%
# Course 6: Capstone Project
# Milestone 1: youBot Kinematics Simulator & CSV output
# Input:
# 1)Initial config of end effector T_se,init
# 2) Initial config of cube T_sc,init
# 3) Final config of cube T_sc,final
# 4) end effector config relative to cube when grasping T_ce,grasp
# 5) end effector standoff distance from cube when not grasping T_ce,standoff
# Output:
# 1) N x13 matrix of robot configurations to move the cube in csv file
#       - each row is a length-13 vector of [r11, r12, r13, r21, r22, r23, r31, r32, r33, px, py, pz, gripper_state]


import numpy as np
import modern_robotics as mr
import csv

Tse_init = np.zeros((4, 4))
Tsc_init = np.zeros((4, 4))
Tsc_final = np.zeros((4, 4))
T_ce_grasp = np.zeros((4, 4))
T_ce_standoff = np.zeros((4, 4))

# def TrajectoryGenerator(Tse_init, Tsc_init, Tsc_final, T_ce_grasp, T_ce_standoff, N):

# make full trajectory matrix ---------------------

Tse_init_standoff = Tsc_init@T_ce_standoff
Tse_init_grasp = Tsc_init@T_ce_grasp

Tse_final_standoff = Tsc_final@T_ce_standoff
Tse_final_grasp = Tsc_final@T_ce_grasp

traj_4x4 = [Tse_init, Tse_init_standoff,
            Tse_init_grasp, Tse_final_standoff, Tse_final_grasp]

traj_8x16 = np.zeros((8, 16))
i = 0
for node in traj_4x4:
    traj_8x16[i, :] = node.reshape(-1)
    i = i+1


# calc
for n in range(1, traj_8x16.shape[0] + 1):

    Xstart = traj_8x16[n, :] #initial config (4X4 SE(3))
    Xend = traj_8x16[n+1, :] #Final configuration (4×4 SE(3) matrix)
    Tf = #Total time of motion (scalar)
    N  = #Number of trajectory points (integer)
    Method = 'cubic'

# trajectory calc

print(traj_8x16.shape)

# return

# %%
