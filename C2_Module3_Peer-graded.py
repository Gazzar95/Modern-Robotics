# %%

from IK_Test import IKinBodyIterates
import numpy as np
import modern_robotics as mr


W1 = 109  # in mm
W2 = 82  # in mm
L1 = 425  # in mm
L2 = 392  # in mm
H1 = 89  # in mm
H2 = 95  # in mm

# home configuration, M
M = np.array([[-1, 0, 0, L1+L2], [0, 0, 1, W1+W2],
             [0, 1, 0, H1-H2], [0, 0, 0, 1]])

# Screw Axises end-effector frame, B
B1 = np.array([0, 1, 0, W1+W2, 0, L1+L2]).reshape(-1, 1)
B2 = np.array([0, 0, 1, H2, -L1-L2, 0]).reshape(-1, 1)
B3 = np.array([0, 0, 1, H2, -L2, 0]).reshape(-1, 1)
B4 = np.array([0, 0, 1, H2, 0, 0]).reshape(-1, 1)
B5 = np.array([0, -1, 0, -W2, 0, 0]).reshape(-1, 1)
B6 = np.array([0, 0, 1, 0, 0, 0]).reshape(-1, 1)

B = np.block([B1, B2, B3, B4, B5, B6])

# Desired end effector configuration, T

T = np.array([[0, 1, 0, -.5],
              [0, 0, -1, 0.1],
              [-1, 0, 0, 0.1],
              [0, 0, 0, 1]])


# error tolerances
eomg = 0.01
ev = 0.01

# initial guess
Thetalist0 = np.array([-0.15, -2.22, -2.1, 0.066, -3.041, 0.463])


# theta2, success2 = mr.IKinBody(B, M, T, Thetalist0, eomg, ev)
theta, success, iterations = IKinBodyIterates(B, M, T, Thetalist0, eomg, ev)
theta2, success2 = mr.IKinBody(B, M, T, Thetalist0, eomg, ev)

# print(success2)
print(success)
print(success2)

with open(iterations, "r") as f:
    log_contents = f.read()
    print("\n--- Iterations Log ---")
    print(log_contents)


# %%
