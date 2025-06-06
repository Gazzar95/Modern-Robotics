# %% Course 1 - Module 1 Assignment
# Problem 1
import numpy as np
import modern_robotics as mr

L = 1

R = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
P = np.array([L+np.sqrt(3)*L+L, 0, -L+np.sqrt(3)*L+2*L]).reshape(-1, 1)

M = np.block([[R, P],
              [0, 0, 0, 1]])


print('M = \n', M.tolist())
# %% - Problem 2

w_1 = np.array((0, 0, 1))
v_1 = np.array((0, -L, 0))
S_1 = np.concatenate((w_1, v_1)).reshape(-1, 1)

w_2 = np.array((0, 1, 0))
v_2 = np.array((0, 0, L))
S_2 = np.concatenate((w_2, v_2)).reshape(-1, 1)

w_3 = np.array((0, 1, 0))
q_3 = np.array((L+np.sqrt(3)*L, 0, -L))
v_3 = -np.cross(w_3, q_3)
S_3 = np.concatenate((w_3, v_3)).reshape(-1, 1)

w_4 = np.array((0, 1, 0))
q_4 = np.array((L+np.sqrt(3)*L+L, 0, -L+np.sqrt(3)*L))
v_4 = -np.cross(w_4, q_4)
S_4 = np.concatenate((w_4, v_4)).reshape(-1, 1)

w_5 = np.array((0, 0, 0))
v_5 = np.array((0, 0, 1))
S_5 = np.concatenate((w_5, v_5)).reshape(-1, 1)

w_6 = np.array((0, 0, 1))
q_6 = np.array((L+np.sqrt(3)*L+L, 0, -L+np.sqrt(3)*L+2*L))
v_6 = -np.cross(w_6, q_6)
S_6 = np.concatenate((w_6, v_6)).reshape(-1, 1)

S = np.block([S_1, S_2, S_3, S_4, S_5, S_6])

print('S is:\n', S.tolist())

# %% - Problem 3

w_1_b = np.array((0, 0, 1))
q_1_b = np.array((-L-np.sqrt(3)*L, 0, L-np.sqrt(3)*L-2*L))
v_1_b = -np.cross(w_1_b, q_1_b)
B_1 = np.concatenate((w_1_b, v_1_b)).reshape(-1, 1)

w_2_b = np.array((0, 1, 0))
q_2_b = np.array((-np.sqrt(3)*L-L, 0, L-np.sqrt(3)*L-2*L))
v_2_b = -np.cross(w_2_b, q_2_b)
B_2 = np.concatenate((w_2_b, v_2_b)).reshape(-1, 1)

w_3_b = np.array((0, 1, 0))
q_3_b = np.array((-L, 0, -np.sqrt(3)*L-2*L))
v_3_b = -np.cross(w_3_b, q_3_b)
B_3 = np.concatenate((w_3_b, v_3_b)).reshape(-1, 1)

w_4_b = np.array((0, 1, 0))
q_4_b = np.array((0, 0, -2*L))
v_4_b = -np.cross(w_4_b, q_4_b)
B_4 = np.concatenate((w_4_b, v_4_b)).reshape(-1, 1)

w_5_b = np.array((0, 0, 0))
v_5_b = np.array((0, 0, 1))
B_5 = np.concatenate((w_5_b, v_5_b)).reshape(-1, 1)

w_6_b = np.array((0, 0, 1))
v_6_b = np.array((0, 0, 0))
B_6 = np.concatenate((w_6_b, v_6_b)).reshape(-1, 1)

B = np.block([B_1, B_2, B_3, B_4, B_5, B_6])

print('B = \n', B.tolist())

# %% Problem 4

Theta = np.array([-(np.pi/2), (np.pi/2), (np.pi/3), -(np.pi/4), 1, (np.pi/6)])

T = mr.FKinSpace(M, S, Theta)

print('T_space = \n', T.tolist())
# %% Problem 5

T = mr.FKinBody(M, B, Theta)

print('T_body = \n', T.tolist())
