# %%
# Course 5; Module 5; Assignment
# Q6. Mobile Manipulator Jacobian
# Find: Je2 - the second column of Jacobian (corresponding to the right wheel)

import numpy as np
import modern_robotics as mr

r = 0.5  # wheel radius
d = 1  # distance from center axis to wheel
c = 2  # distance from chassis to robot joint
k = 3  # length of robot link
angle = np.pi/2  # angle of the robot joint

F = r/2*np.array([[-1/d, 1/d],
                  [1, 1],
                  [0, 0]])

F_6 = np.zeros((6, 2))
F_6[2:5, :] = F

print("F_6 =\n", F_6)

J_base = np.transpose(mr.Adjoint(F_6))

print("J_base =\n", J_base)
# %%
