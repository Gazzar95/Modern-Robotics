# Course 6: Capstone Project
# Milestone 3: youBot Kinematics Simulator & CSV output
# Input:
# 1)End effector configuration T_se
# 2)End effector ref (desired) config T_se,d
# 3)End effector config in the next time step T_se,next
# 4) PI controller gains Kp, Ki
# 5) time step dt
# Output:
# 1)End effector velocity command V_se

import numpy as np
import modern_robotics as mr


def FeedbackControl(Tse, Tse_d, Tse_d_next, Kp, Ki, dt, X_err_int=np.zeros(6)):
    """
    Computes the commanded end-effector twist V (6×1) and the current error twist X_err (6×1)
    for the Modern Robotics Milestone 3 feedback controller.

    Inputs:
    - Tse:           Actual end-effector configuration (4×4 homogeneous matrix)
    - Tse_d:         Desired end-effector configuration at current timestep (4×4)
    - Tse_d_next:    Desired end-effector configuration at next timestep (4×4)
    - Kp:            6×6 proportional gain matrix
    - Ki:            6×6 integral gain matrix
    - dt:            Time step (seconds)
    - X_err_int:     6×1 vector for accumulated integral error (default zeros)

    Outputs:
    - V:             6×1 commanded body twist (in current end-effector frame)
    - X_err:         6×1 instantaneous error twist (body frame)
    - X_err_int_new: 6×1 updated integral of error
    """

    # 1. Compute body-frame configuration error
    T_err = mr.TransInv(Tse) @ Tse_d
    X_err = mr.se3ToVec(mr.MatrixLog6(T_err))

    # 2. Update integral of error
    X_err_int_new = X_err_int + X_err * dt

    # 3. Compute desired body twist (feedforward term)
    Vd_d = (1 / dt) * mr.se3ToVec(mr.MatrixLog6(mr.TransInv(Tse_d) @ Tse_d_next))

    # Transform desired twist from {d} frame to actual end-effector frame {e}
    Vd_e = mr.Adjoint(mr.TransInv(Tse) @ Tse_d) @ Vd_d

    # 4. PI control law
    V = Vd_e + Kp @ X_err + Ki @ X_err_int_new

    return V, X_err, X_err_int_new
