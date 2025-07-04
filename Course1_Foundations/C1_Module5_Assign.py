# Course 1 - Module 5 Assignment
# %%Problem 1 & 2 ---------------------------------------------------------------
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

print("prob 1 T_sa:\n", T_sa.tolist())

T_sb_inverse = np.linalg.inv(T_sb)

print('prob 2 T_sb_inverse:\n', T_sb_inverse.tolist())

# %% Problem 3 ---------------------------------------------------------------

T_sa = np.array([[0, -1, 0, 0], [0, 0, -1, 0], [1, 0, 0, 1], [0, 0, 0, 1]])

T_sb = np.array([[1, 0, 0, 0], [0, 0, 1, 2], [0, -1, 0, 0], [0, 0, 0, 1]])

T_sa_inverse = np.linalg.inv(T_sa)

T_ab = np.matmul(T_sa_inverse, T_sb)

print('prob 3 T_ab:\n', T_ab.tolist())

# %% Problem 5 ---------------------------------------------------------------

# Define the transformation matrix T_sb
T_sb = np.array([[1, 0, 0, 0],       # Rotation and translation elements for T_sb
                 # The second row indicates a rotation and translation along the z-axis
                 [0, 0, 1, 2],
                 # The third row indicates a rotation and no translation in the z-direction
                 [0, -1, 0, 0],
                 # The last row ensures the matrix is in homogeneous coordinates
                 [0, 0, 0, 1]])

# Define the point p_b in homogeneous coordinates (4D)
# The last element is 1 to represent homogeneous coordinates
p_b = np.array([1, 2, 3, 1])

# Reshape p_b to a column vector (4x1)
p_b = p_b.reshape(-1, 1)

# Apply the transformation T_sb to the point p_b using matrix multiplication
p_s = np.dot(T_sb, p_b)  # This computes the transformed point p_s

# Print the transformed point p_s in list format for readability
# Convert to list for easier viewing of the result
print('prob 5 p_s:\n', p_s.tolist())

# %% Problem 7 ---------------------------------------------------------------

V_s = np.array([3, 2, 1, -1, -2, -3])
T_sa = np.array([[0, -1, 0, 0], [0, 0, -1, 0], [1, 0, 0, 1], [0, 0, 0, 1]])

T_sa_inverse = mr.TransInv(T_sa)

Ad_T_sa = mr.Adjoint(T_sa_inverse)

V_a = np.dot(Ad_T_sa, V_s)

print('prob 7 V_a:\n', V_a.tolist())

# %% Problem 8 --------------------------------------------------------------

T_sa = np.array([[0, -1, 0, 0], [0, 0, -1, 0], [1, 0, 0, 1], [0, 0, 0, 1]])

S_theta = mr.MatrixLog6(T_sa)

print('prob 8 S_theta:\n', S_theta.tolist())

# %% Problem 9 --------------------------------------------------------------

# Define a screw vector (6D)
# Make sure this is a 1D array with 6 elements
S_theta = np.array([0, 1, 2, 3, 0, 0, 0])

S_Theta_Skew = mr.VecTose3(S_theta)

# Compute the transformation matrix from the screw vector
T = mr.MatrixExp6(S_Theta_Skew)

# Print the resulting transformation matrix
print('prob 9 T:\n', T.tolist())

# %% Problem 10 --------------------------------------------------------------

F_b = np.array([1, 0, 0, 2, 1, 0])

T_sb = np.array([[1, 0, 0, 0], [0, 0, 1, 2], [0, -1, 0, 0], [0, 0, 0, 1]])

T_bs = np.transpose(T_sb)

Ad_T_bs = mr.Adjoint(T_sb)

Ad_T_sb = np.transpose(Ad_T_bs)

F_s = np.dot(Ad_T_sb, F_b)

print('prob 10 F_s:\n', F_s.tolist())

# %% Problem 11 --------------------------------------------------------------

T = np.array([[0, -1, 0, 3], [1, 0, 0, 0], [0, 0, 1, 1], [0, 0, 0, 1]])

T_inv = mr.TransInv(T)

print('prob 11 T_inv:\n', T_inv.tolist())

# %% problem 13 --------------------------------------------------------------

# Define the screw axis
s_hat = np.array([1, 0, 0])  # Axis of rotation (3x1)

# Define the point q
q = np.array([0, 0, 2])      # Point in space (3x1)

# Define the pitch h
h = 1                        # Pitch (1x1)

# Calculate the cross product for the linear velocity component
v1 = np.cross(s_hat, q)      # Resulting vector from the cross product (3x1)

# Calculate the linear velocity component scaled by pitch
v2 = h * s_hat               # Linear velocity component (3x1)

# Combine into a single 6x1 screw vector
# Concatenate s_hat and the resulting linear velocity
S1 = np.concatenate((s_hat, v1 + v2))

# Print the resulting 6x1 vector
print("Screw vector S:\n", S1)  # Reshape to 6x1 for clarity

S2 = mr.ScrewToAxis(q, s_hat, h)

print("Screw vector S wth function ScrewToAxis:\n", S2)

# %% Problem 14 ---------------------------------------------------------------


S_Theta = np.array([[0, -1.5708, 0, 2.3562],
                   [1.5708, 0, 0, -2.3562],
                   [0, 0, 0, 1],
                   [0, 0, 0, 0]])

T = mr.MatrixExp6(S_Theta)

print(T.tolist())

# %% problem 15 ---------------------------------------------------------------


T = np.array([[0, -1, 0, 3],
              [1, 0, 0, 0],
              [0, 0, 1, 1],
              [0, 0, 0, 1]])

S_theta = mr.MatrixLog6(T)

print(S_theta.tolist())

# %%
