# %%
# Course 5; Module 5; Assignment
# Q6 & Q7. Mobile Manipulator Jacobian
# Find: Je2 - the second column of Jacobian (corresponding to the right wheel)

import numpy as np
import modern_robotics as mr

r = 0.5  # wheel radius
d = 1  # distance from center axis to wheel
c = 2  # distance from chassis to robot joint
k = 3  # length of robot link
angle = np.pi/2  # angle of the robot joint

# ----------Calc J_Base----------

# Build F (or H(0)) - wheel to chassis
F = r/2*np.array([[-1/d, 1/d],
                  [1, 1],
                  [0, 0]])

F_6 = np.zeros((6, 2))
F_6[2:5, :] = F

# Build T_b0 - chassis to base of robot joint
R_b0 = np.eye(3)
p_b0 = np.array([c, 0.0, 0.0])   # assume pure x translation
T_b0 = mr.RpToTrans(R_b0, p_b0)

# Build T_0e - base of robot joint to end-effector
ca, sa = np.cos(angle), np.sin(angle)
R_0e = np.array([[ca, -sa, 0.0],
                 [sa,  ca, 0.0],
                 [0.0, 0.0, 1.0]])
p_0e = R_0e @ np.array([k, 0.0, 0.0])
T_0e = mr.RpToTrans(R_0e, p_0e)

# Compute T_be and its Adjoint - chassis to end-effector
T_be = np.linalg.inv(T_0e) @ np.linalg.inv(T_b0)
Ad = mr.Adjoint(T_be)

J_base = Ad @ F_6

# ----------Calc J_Arm----------

S1 = np.array([0, 0, 1, 0, 0, 0])
T_e0 = np.linalg.inv(T_0e)
J_arm = mr.Adjoint(T_e0) @ S1

print("J_base =\n", repr(J_base))  # second column of Jacobian
print("J_arm =\n", repr(J_arm))  # arm Jacobian

# %%
