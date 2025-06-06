# %% Problem 1
import numpy as np
import modern_robotics as mr

# Initialize variables
x = 1
y = 1
n = 1
i = 4

# Create an empty array to store theta results
Theta_results = np.empty((2, 0))

# Initialize Theta based on the given equations
Theta = np.array([[x], [y]], dtype=float)

# Perform the iterative process to update Theta
while n < i:
    # Append the current Theta to the results
    Theta_results = np.column_stack((Theta_results, Theta))

    # Define the function f based on current x and y
    f = np.array([[x ** 2 - 9], [y ** 2 - 4]])

    # Compute the Jacobian matrix J
    J = np.array([[2*x, 0],
                  [0, 2*y]])

    # Update Theta using the Newton-Raphson method
    Theta = Theta - np.linalg.inv(J) @ f

    # Update x and y with the new values from Theta
    x = Theta[0, 0]
    y = Theta[1, 0]

    # Increment the iteration count
    n += 1

# Print the final results of Theta calculations
print('theta results = \n', Theta_results)

# %% Problem 2

# Define screw axes for the robot joints
w_1 = np.array((0, 0, 1))  # Rotation axis for joint 1
v_1 = np.array((0, 0, 0))  # Linear velocity vector for joint 1
S_1 = np.concatenate((w_1, v_1)).reshape(-1, 1)  # Combine w and v for joint 1

w_2 = np.array((0, 0, 1))  # Rotation axis for joint 2
v_2 = np.array((0, -1, 0))  # Linear velocity vector for joint 2
S_2 = np.concatenate((w_2, v_2)).reshape(-1, 1)  # Combine w and v for joint 2

w_3 = np.array((0, 0, 1))  # Rotation axis for joint 3
v_3 = np.array((0, -2, 0))  # Linear velocity vector for joint 3
S_3 = np.concatenate((w_3, v_3)).reshape(-1, 1)  # Combine w and v for joint 3

# Combine all screw vectors into a single matrix S
S = np.block([S_1, S_2, S_3])

# Define the transformation matrix M for the robot
M = np.array([[1, 0, 0, 3],
              [0, 1, 0, 0],
              [0, 0, 1, 0],
              [0, 0, 0, 1]])

# Define the transformation matrix T for the end effector
T = np.array([[-.585, -.811, 0, .076],
              [.811, -.585, 0, 2.608],
              [0, 0, 1, 0],
              [0, 0, 0, 1]])

# Initial guess for the joint angles
Thetalist0 = np.array([0.7854, 0.7854, 0.7854])

# Define tolerances for the inverse kinematics
eomg = 0.001  # Tolerance for orientation
ev = 0.0001   # Tolerance for position

# Perform inverse kinematics to find the joint angles
[Thetalist, success] = mr.IKinSpace(S, M, T, Thetalist0, eomg, ev)

# Print the joint angles
print('Joint angles = \n', Thetalist)
print('Success = \n', success)
# %%
