# %%
# 4/12/2025 - 3:37 PM
# @author: Omar El Gazzar
# Course 1, Module 4, Grade Assignment

import modern_robotics as mr
import numpy as np

R = np.array([[0, 0.5, -1], [-0.5, 0, 2], [1, -2, 0]])

#R = mr.VecToso3(R)

w_s_1 = mr.MatrixExp3(R)

print(w_s_1)


# %%
