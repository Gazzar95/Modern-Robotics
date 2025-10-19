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

# ----------- Calc tot time of motion Tf fer each segment --------------------


def segment_duration(Xstart, Xend, v_max=0.1, w_max=1.0):
    """Estimate reasonable Tf for ScrewTrajectory segment."""

    # Euclidean distance distance
    dp = np.linalg.norm(Xend[0:3, 3] - Xstart[0:3, 3])

    # rotation difference (angle)
    R_rel = Xstart[0:3, 0:3].T @ Xend[0:3, 0:3]
    omg, theta = mr.so3ToVec(mr.MatrixLog3(R_rel)), np.linalg.norm(
        mr.so3ToVec(mr.MatrixLog3(R_rel)))

    Tf = max(dp / v_max, theta / w_max)

    # Round up to nearest 0.01 s
    Tf = np.ceil(Tf / 0.01) * 0.01
    return Tf

# *********************** Trouble shoot values ************************************


Tse_init = np.zeros((4, 4))
Tsc_init = np.zeros((4, 4))
Tsc_final = np.zeros((4, 4))
T_ce_grasp = np.zeros((4, 4))
T_ce_standoff = np.zeros((4, 4))

v_max = 0.1 #max linear velocity
w_max = 0.1 #max angular velocity

method = 5

# def TrajectoryGenerator(Tse_init, Tsc_init, Tsc_final, T_ce_grasp, T_ce_standoff, N):

# ------------------- Combine confgs into a single trajectory matrix ---------------------

# change standoff & grip position to be rel to space frame
Tse_init_standoff = Tsc_init@T_ce_standoff
Tse_init_grasp = Tsc_init@T_ce_grasp

Tse_final_standoff = Tsc_final@T_ce_standoff
Tse_final_grasp = Tsc_final@T_ce_grasp

# 8 segments: (start, end, is_hold, gripper_state_during_segment)
segments = [
     (Tse_init,           Tse_init_standoff,  False, 0),  # 1: move open
     (Tse_init_standoff,  Tse_init_grasp,     False, 0),  # 2: move open
     (Tse_init_grasp,     Tse_init_grasp,     True,  1),  # 3: hold (close -> 1)
     (Tse_init_grasp,     Tse_init_standoff,  False, 1),  # 4: move closed
     (Tse_init_standoff,  Tse_final_standoff, False, 1),  # 5: transit closed
     (Tse_final_standoff, Tse_final_grasp,    False, 1),  # 6: move closed
     (Tse_final_grasp,    Tse_final_grasp,    True,  0),  # 7: hold (open -> 0)
     (Tse_final_grasp,    Tse_final_standoff, False, 0),  # 8: move open
]



for Xstart, Xend, is_hold, g in segments:
    
    if is_hold:
       N = np.ceil(hold_time / dt)
       Tf = float(N * dt)

    else:
     dt = 0.01 #time step

     # Total time of motion (scalar)
     Tf = segment_duration(Xstart, Xend, v_max, w_max)
     N =  N = int(Tf / dt) # Number of trajectory points (integer)
     
     Traj = mr.ScrewTrajectory(Xstart, Xend, Tf, N, method)

# trajectory calc

print(traj_8x16.shape)

# return

# %%
