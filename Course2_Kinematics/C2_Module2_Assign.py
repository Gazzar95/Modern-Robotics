# %% Problem 1 ---------------------------------------------------------------
# Problem: Finld the joint torques (Tao) given the end effector force (Fx, Fy)
import numpy as np
import modern_robotics as mr

f_x_s = 2  # Newtons

f_s = np.array([f_x_s, 0, 0]).reshape(-1, 1)

# Calc Home Position Transformation Matrix
M = np.array([[1, 0, 0, 3], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

# calculate screw vector for 3 joints
S_1 = np.array([0, 0, 1, 0, 0, 0]).reshape(-1, 1)
S_2 = np.array([0, 0, 1, 0, -1, 0]).reshape(-1, 1)
S_3 = np.array([0, 0, 1, -0.7071, 1.7071, 0]).reshape(-1, 1)

S = np.column_stack((S_1, S_2, S_3))

# calculate joint angles
Theta = np.array([0, np.deg2rad(45), np.deg2rad(-45)]).reshape(-1, 1)

# calculate Jacobian
J = mr.JacobianSpace(S, Theta)

print('Jacobian = :\n', J)

# calculate joint torques

tau = np.dot(J.T, f_s)

print('tau = :\n', tau)
# %% Problem 2 ---------------------------------------------------------------

L = 1
Theta1 = 0
Theta2 = 0
Theta3 = np.pi/2
theta4 = -np.pi/2

Theta = np.array([Theta1, Theta2, Theta3, theta4]).reshape(-1, 1)

F = np.array([0, 0, 0, 10, 10, 0]).reshape(-1, 1)

# calculate S
S_1 = np.array([0, 4*L, 0, -4*L, 0, 0]).reshape(-1, 1)
S_2 = np.array([0, 3*L, 0, -3*L, 0, 0]).reshape(-1, 1)
S_3 = np.array([0, 2*L, 0, -2*L, 0, 0]).reshape(-1, 1)
S_4 = np.array([0, L, 0, -L, 0, 0]).reshape(-1, 1)

S = np.column_stack((S_1, S_2, S_3, S_4))

# calculate J
J = mr.JacobianBody(S, Theta)

tau = np.dot(J.T, F)

print('tau = :\n', tau)
# %%

# Joint angles
theta1 = 0
theta2 = 0
theta3 = np.pi / 2
theta4 = -np.pi / 2

# Link lengths
L1 = L2 = L3 = L4 = 1

# Compute sine values for shorthand terms
s23 = np.sin(theta2 + theta3)       # sin(π/2) = 1
s234 = np.sin(theta2 + theta3 + theta4)  # sin(0) = 0
s34 = np.sin(theta3 + theta4)       # sin(0) = 0

# Now build the Jacobian based on the simplified matrix from the image
# The Jacobian provided corresponds to the planar body Jacobian Jb_planar:
J_planar = np.array([
    [1 / (L4 + L3 * s4 + L2 * s34 + L1 * s234) if denom != 0 else 0 for s4, s34, s234, denom in
     zip([0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [4, 3, 2, 1])],  # row 1: ω_z
    [1, 1, 0, 0],  # row 2: v_x
    [1, 0, 0, 0]   # row 3: v_y
])

# But we can hardcode this easily for this specific configuration:
# As evaluated previously:
J_planar = np.array([
    [1/4, 1/3, 1/2, 1],
    [1,   1,   0,   0],
    [1,   0,   0,   0]
])

# Planar wrench in the body frame: [Mz, Fx, Fy]
F_planar = np.array([0, 10, 10]).reshape(3, 1)

# Compute joint torques
tau = J_planar.T @ F_planar

print("Joint torques (planar method):")
print(tau)

# %% problem 3

S_1 = np.array([0, 0, 1, 0, 0, 0]).reshape(-1, 1)
S_2 = np.array([1, 0, 0, 0, 2, 0]).reshape(-1, 1)
S_3 = np.array([0, 0, 0, 0, 1, 0]).reshape(-1, 1)

S = np.column_stack((S_1, S_2, S_3))

Theta = np.array([np.deg2rad(90), np.deg2rad(90), 1]).reshape(-1, 1)

J = mr.JacobianSpace(S, Theta)

print('J = \n', J.tolist())
# %% problem 4

S_1 = np.array([0, 1, 0, 3, 0, 0]).reshape(-1, 1)
S_2 = np.array([-1, 0, 0, 0, 3, 0]).reshape(-1, 1)
S_3 = np.array([0, 0, 0, 0, 0, 1]).reshape(-1, 1)

S = np.column_stack((S_1, S_2, S_3))

Theta = np.array([np.deg2rad(90), np.deg2rad(90), 1]).reshape(-1, 1)

J = mr.JacobianBody(S, Theta)

print('J = \n', J.tolist())

# %% problem 5


# Step 1: Define J_v (bottom 3 rows of J_b)
J_v = np.array([
    [-0.105, 0,     0.006, -0.045, 0,     0.006, 0],
    [-0.889, 0.006, 0,     -0.844, 0.006, 0,     0],
    [0,     -0.105, 0.889, 0,      0,     0,     0]
])

# Step 2: Compute A = J_v * J_v^T
A = J_v @ J_v.T

# Step 3: Compute eigenvalues and eigenvectors
eigenvalues, eigenvectors = np.linalg.eigh(A)

# Step 4: Find the eigenvector corresponding to the largest eigenvalue
max_index = np.argmax(eigenvalues)
max_eigenvector = eigenvectors[:, max_index]

# Normalize the vector (to ensure it's a unit vector)
unit_vector = max_eigenvector / np.linalg.norm(max_eigenvector)

# Output result
print("Direction of longest principal semi-axis (unit vector):")
print(np.round(unit_vector, 4))
