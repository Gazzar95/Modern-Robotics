# %%
# Capstone Main Script
# Course 6: Capstone Project
# Milestone 4: youBot Kinematics Simulator & CSV output


import numpy as np
import modern_robotics as mr
import csv
from TrajectoryGenerator import TrajectoryGenerator
from FeedbackControl import FeedbackControl
from NextState import NextState

# Main function


def main():

    # Trajectory Variables
    Tse_init = np.array([
        [0, 0, 1, 0.5],
        [0, 1, 0, 0.0],
        [-1, 0, 0, 0.5],
        [0, 0, 0, 1]
    ])

    Tsc_init = np.array([
        [1, 0, 0, 1.0],
        [0, 1, 0, 0.0],
        [0, 0, 1, 0.025],
        [0, 0, 0, 1]
    ])

    Tsc_final = np.array([
        [0, 1, 0, 0.0],
        [-1, 0, 0, -1.0],
        [0, 0, 1, 0.025],
        [0, 0, 0, 1]
    ])

    Tce_grasp = np.array([
        [0, 0, 1, 0.0],
        [0, 1, 0, 0.0],
        [-1, 0, 0, 0.015],
        [0, 0, 0, 1]
    ])

    Tce_standoff = np.array([
        [0, 0, 1, 0.0],
        [0, 1, 0, 0.0],
        [-1, 0, 0, 0.05],
        [0, 0, 0, 1]
    ])

    # youBot Kinematics Variables
    M0e = np.array([
        [1, 0, 0, 0.033],
        [0, 1, 0, 0.0],
        [0, 0, 1, 0.6546],
        [0, 0, 0, 1]
    ])  # home config of end-effector
    Blist = np.array([
        [0, 0, 1, 0, 0.033, 0],
        [0, -1, 0, -0.5076, 0, 0],
        [0, -1, 0, -0.3526, 0, 0],
        [0, -1, 0, -0.2176, 0, 0],
        [0, 0, -1, 0, 0, 0]
    ]).T  # body screw axes

    # Simulation Variables
    # initial joint and wheel angles
    current_config = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    joint_speeds = np.zeros(9)  # initial joint speeds
    dt = 0.01  # time step
    joint_limits = np.array([
        [-2.9671, 2.9671],  # joint 1 limits
        [-1.8326, 1.8326],  # joint 2 limits
        [-2.9671, 2.9671],  # joint 3 limits
        [-3.7525, -0.0698],  # joint 4 limits
        [-2.9671, 2.9671]   # joint 5 limits
    ])

    # Generate Trajectory
    Traj = TrajectoryGenerator(
        Tse_init, Tsc_init, Tsc_final, Tce_grasp, Tce_standoff)

    for row in Traj:

        # Create Tse_d, Tse_d_next for testing FeedbackControl
        Tse_d = mr.MatrixFromList(row[0:16]).reshape((4, 4))
        gripper_state = row[16]

        Tse_d_next = mr.MatrixFromList(
            Traj[row + 1][0:16]).reshape((4, 4)) if row + 1 < len(Traj) else Tse_d

        if row == 0:
            current_config = np.zeros(12)  # initial full config
            Tse = Tse_init  # initial actual config
        else:
            current_config = NextState(
                current_config, joint_speeds, dt, joint_limits)
            # compute actual end-effector config
            Tse = mr.FKinBody(M0e, Blist, current_config[3:8])

        # Log data to CSV
        with open('youBot_Simulation_Log.csv', mode='a', newline='') as file:
            writer = csv.writer(file)
            row_data = list(current_config) + \
                list(Tse.flatten()) + [gripper_state]
            writer.writerow(row_data)

        # Compute feedback control
        Kp = np.eye(6) * 1.0
        Ki = np.eye(6) * 0.0
        V, X_err, X_err_int_new = FeedbackControl(
            Tse, Tse_d, Tse_d_next, Kp, Ki, dt)

        # Jacobian
        J_arm = mr.JacobianBody(Blist, current_config[3:8])
        # Wheel Jacobian
        r = 0.0475  # wheel radius
        l = 0.235   # half distance along x
        w = 0.15    # half distance along y
        H0 = (1.0 / r) * np.array([
            [-l - w,  1, -1],
            [l + w,  1,  1],
            [l + w,  1, -1],
            [-l - w,  1,  1]
        ])  # shape (4,3)
        F = np.linalg.pinv(H0)                 # 3x4
        F_6 = np.zeros((6, 4))
        F_6[2:5, :] = F
        T_0e = mr.FKinBody(M0e, Blist, current_config
                           [3:8])
        T_eb = np.linalg.inv(T_0e)
        Ad = mr.Adjoint(T_eb)
        J_base = Ad @ F_6
        J = np.hstack((J_base, J_arm))  # full Jacobian

        # Compute joint speeds
        joint_speeds = np.linalg.pinv(J) @ V
