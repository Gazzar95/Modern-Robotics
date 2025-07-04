# %%
# Module 5 Assignment

import numpy as np
import modern_robotics as mr

# calc ref frames a, & b's vector Z
x_a = [0, 0, 1]  # x vector of frame a in relation to s
y_a = [-1, 0, 0]  # y vector of frame a in relation to s

x_b = [1, 0, 0]  # x vector of frame b in relation to s
y_b = [0, 0, -1]  # y vector of frame b in relation to s

z_a = np.cross(x_a, y_a)
z_b = np.cross(x_b, y_b)

# Value of p
p_a = np.array([0, 0, 1])  # p vector of frame a in relation to s
p_b = np.array([0, 2, 0])  # p vector of frame b in relation to s

p_a = p_a.reshape(-1, 1)
p_b = p_b.reshape(-1, 1)

# print("Z for ref frame a is:", z_a)
# print("Z for ref frame b is:", z_b)

# Calculate Rotation matrix
R_sa = np.transpose([x_a, y_a, z_a])
R_sb = np.transpose([x_b, y_b, z_b])

# Calculate Transformation Matrix
T_sa = np.block([[R_sa, p_a],
                 [0, 0, 0, 1]])
T_sb = np.block([[R_sb, p_b],
                [0, 0, 0, 1]])


# Print the transformation matrices
# print('Tsa =\n', T_sa)
# print('Tsb =\n', T_sb)

S_Theta = mr.MatrixLog6(T_sa)

# print('S_Theta =\n', S_Theta)

# prob 9
S_Theta_prob9 = np.array([0, 1, 2, 3, 0, 0])

S_Theta2 = mr.VecTose3(S_Theta_prob9)
print('S_Theta2 =\n', S_Theta2)

T_prob9 = mr.MatrixExp6(S_Theta)

print('S_Theta_prob9 =\n', S_Theta_prob9)
print('S_Theta =\n', S_Theta)
# formatted_T_prob9 = np.array2string(T_prob9, bracket='()', separator=', ', precision=2, suppress_small=True)

# print('T =\n',formatted_T_prob9)

T = np.array([[0, -1, 0, 3], [1, 0, 0, 0], [0, 0, 1, 1], [0, 0, 0, 1]])

T_inv = mr.TransInv(T)

v = np.array([1, 0, 0, 0, 2, 3])

# Convert the vector to a homogeneous transformation matrix
T = mr.VecTose3(v)

# print("Homogeneous transformation matrix:\n", T)

# %%
