# %% Peer Graded Assignment C3 Module 2
# A. Goal: 
# Using robot dynamics principles to write a script that simulates the motion of the UR5 falling due to gravity.
# B. Simulations:
# 1) The robot falling from the zero configuration for 3 seconds
# 2) The robot falling from a configuration with all joints at their zero position, except for joint 2, which is at -1 rad. This sim should last 5 seconds.
# C. Requirements:
# 100 steps/second
# output .csv file of angles for each step

import numpy as np
import modern_robotics as mr
from FD_IterationFunction import ForwardDynamicsIterate

# Simulation duration
total_time = 5  # seconds for the first simulation
steps_or_second = 100  # steps per second
dt = 1 / steps_or_second  # time step in seconds
N  = int(total_time/dt)  # number of steps
n_joints = 6
intRes = 1  # Integration resolution

taumat = np.zeros((N+1, n_joints))  # joint torques matrix
Ftipmat = np.zeros_like(taumat)  # end-effector forces matrix

# Initial joint angles, velocities
thetalist = np.array([0, -1, 0, 0, 0, 0])  # Joint angles in radians
dthetalist = np.array([0, 0, 0, 0, 0, 0])  # Joint velocities in rad/s

# Gravity vector and end-effector force
g = np.array([0, 0, -9.81])  # m/s^2 Gravity vector
Ftip = np.array([0, 0, 0, 0, 0, 0])

#Kinematic and inertial parameters for the UR5 robot
M01 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.089159], [0, 0, 0, 1]]
M12 = [[0, 0, 1, 0.28], [0, 1, 0, 0.13585], [-1, 0, 0, 0], [0, 0, 0, 1]]
M23 = [[1, 0, 0, 0], [0, 1, 0, -0.1197], [0, 0, 1, 0.395], [0, 0, 0, 1]]
M34 = [[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0.14225], [0, 0, 0, 1]]
M45 = [[1, 0, 0, 0], [0, 1, 0, 0.093], [0, 0, 1, 0], [0, 0, 0, 1]]
M56 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.09465], [0, 0, 0, 1]]
M67 = [[1, 0, 0, 0], [0, 0, 1, 0.0823], [0, -1, 0, 0], [0, 0, 0, 1]]
G1 = np.diag([0.010267495893, 0.010267495893,  0.00666, 3.7, 3.7, 3.7])
G2 = np.diag([0.22689067591, 0.22689067591, 0.0151074, 8.393, 8.393, 8.393])
G3 = np.diag([0.049443313556, 0.049443313556, 0.004095, 2.275, 2.275, 2.275])
G4 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
G5 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
G6 = np.diag([0.0171364731454, 0.0171364731454, 0.033822, 0.1879, 0.1879, 0.1879])
Glist = [G1, G2, G3, G4, G5, G6]
Mlist = [M01, M12, M23, M34, M45, M56, M67] 
Slist = [[0,         0,         0,         0,        0,        0],
         [0,         1,         1,         1,        0,        1],
         [1,         0,         0,         0,       -1,        0],
         [0, -0.089159, -0.089159, -0.089159, -0.10915, 0.005491],
         [0,         0,         0,         0,  0.81725,        0],
         [0,         0,     0.425,   0.81725,        0,  0.81725]]

# Simulate the robot falling from the specified configuration
ForwardDynamicsIterate(thetalist, dthetalist, taumat, g, Ftipmat, Mlist, Glist, Slist, dt, intRes)


# %%
