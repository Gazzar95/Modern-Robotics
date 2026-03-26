# %%
# Capstone Main Script
# Course 6: Capstone Project
# Milestone 4: youBot Kinematics Simulator & CSV output


import numpy as np
import modern_robotics as mr
import csv
import os
from TrajectoryGenerator import TrajectoryGenerator
from FeedbackControl import FeedbackControl
from NextState import NextState


# =========================
# Helpers
# =========================
def row13_to_T(row13):
    """Convert [r11..r33, px,py,pz, g] -> (T (4x4), g int)."""
    R = np.array(row13[0:9]).reshape(3, 3)
    p = np.array(row13[9:12]).reshape(3, 1)
    T = np.block([[R, p],
                  [np.zeros((1, 3)), np.array([[1]])]])
    g = int(round(row13[12]))
    return T, g


def Tsb_from_chassis(phi, x, y):
    c, s = np.cos(phi), np.sin(phi)
    return np.array([
        [c, -s, 0, x],
        [s,  c, 0, y],
        [0,  0, 1, 0],
        [0,  0, 0, 1]
    ])


def build_base_F(r, l, w):
    """
    MR canonical 3x4 mapping from wheel rates to chassis body twist [ωz, vx, vy]^T.
    Returns F (3x4). We'll lift to 6x4 as F6 below.
    """
    return (r/4.0) * np.array([
        [-1/(l+w),  1/(l+w),  1/(l+w), -1/(l+w)],
        [1,         1,         1,         1],
        [-1,         1,        -1,         1]
    ])


def whole_robot_body_jacobian(q, M0e, Blist, Tb0, r, l, w):
    """
    Build the 6x9 body Jacobian at the end-effector frame {e}:
    Je = [ Jbase (6x4) | Jarm (6x5) ].
    u is ordered as [wheel1..wheel4, joint1..joint5].
    """
    thetalist = q[3:8]
    T0e = mr.FKinBody(M0e, Blist, thetalist)  # {0}->{e}

    # Base mapping in chassis body frame -> lift to 6x4 twist in {b}
    # 3x4 maps wheel rates -> [ωz, vx, vy]
    F = build_base_F(r, l, w)
    F6 = np.zeros((6, 4))
    F6[2, :] = F[0, :]   # ωz row
    F6[3, :] = F[1, :]   # vx row
    F6[4, :] = F[2, :]   # vy row
    # Transform base twist to end-effector frame {e}: Ad_{Teb} where Teb = (Tb0 @ T0e)^(-1)
    Teb = np.linalg.inv(Tb0 @ T0e)
    Jbase = mr.Adjoint(Teb) @ F6

    # Arm Jacobian in body frame {e}
    Jarm = mr.JacobianBody(Blist, thetalist)          # 6x5

    return np.hstack([Jbase, Jarm])                   # 6x9


# =========================
# Constants (youBot params)
# =========================
# Arm home (M0e) and body screw axes (Blist) per MR youBot
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
    [0, 0, 1, 0, 0, 0]
]).T  # body screw axes

# Transform from chassis frame {b} to arm base frame {0}
Tb0 = np.array([
    [1, 0, 0, 0.1662],
    [0, 1, 0, 0.0],
    [0, 0, 1, 0.0026],
    [0, 0, 0, 1]
])

# Wheel geometry
r = 0.0475
l = 0.235
w = 0.15

# Controller gains (start with Ki=0; add small Ki if you see steady-state error)
Kp = .5 * np.eye((6))
Ki = 0 * np.eye((6))

# Actuator speed limits (example)
wheel_speed_max = 10.0  # rad/s
joint_speed_max = 1.0   # rad/s


# =========================
# Main
# =========================
def main():

    # Trajectory Variables
    Tse_init = Tb0 @ M0e

    Tsc_init = np.array([  # Cube initial position
        [1, 0, 0, 1.0],
        [0, 1, 0, 0.0],
        [0, 0, 1, 0.025],
        [0, 0, 0, 1]
    ])

    Tsc_final = np.array([  # Cube final position
        [0, 1, 0, 0.0],
        [-1, 0, 0, -1.0],
        [0, 0, 1, 0.025],
        [0, 0, 0, 1]
    ])

    Tce_grasp = np.array([  # Grasp position
        [0, 0, 1, 0.0],
        [0, 1, 0, 0.0],
        [-1, 0, 0, -0.075],
        [0, 0, 0, 1]
    ])

    Tce_standoff = np.array([  # Standoff position
        [0, 0, 1, 0.0],
        [0, 1, 0, 0.0],
        [-1, 0, 0, 0.05],
        [0, 0, 0, 1]
    ])

    # ----- Timing -----
    dt = 0.01

    # ----- Build reference trajectory (Nx13 rows) -----
    traj = TrajectoryGenerator(
        Tse_init, Tsc_init, Tsc_final, Tce_grasp, Tce_standoff, dt=dt
    )  # expect shape [N, 13] or list of length N

    # ----- Initialize robot state q (12-vector): [phi, x, y, θ1..θ5, wheel1..wheel4] -----
    q = np.array([
        0.0,  # phi
        0.0,  # x
        0.0,  # y
        0.0,  # θ1
        0.0,  # θ2
        0.2,  # θ3   <-- important
        -1.6,  # θ4   <-- important
        0.0,  # θ5
        0.0, 0.0, 0.0, 0.0  # wheels
    ], dtype=float)
    Xerr_int = np.zeros(6)

    # ----- Open CSV log (optional) -----
    # Keep the files open for the entire control loop so writers are valid

    with open("youBot_Simulation.csv", "w", newline="") as f:
        writer = csv.writer(f)

        # # header (optional)
        # writer.writerow([
        #     "phi", "x", "y", "th1", "th2", "th3", "th4", "th5", "wh1", "wh2", "wh3", "wh4", "g"
        # ])

        # ----- Control loop -----
        for k in range(len(traj) - 1):
            # refs now/next
            Tse_d, g = row13_to_T(traj[k])
            Tse_d_next, _ = row13_to_T(traj[k+1])

            # actual pose from state: Tse = Tsb(q_base) * Tb0 * FKinBody(M0e,Blist, thetalist)
            phi, x, y = q[0], q[1], q[2]
            thetalist = q[3:8]
            Tsb = Tsb_from_chassis(phi, x, y)
            T0e = mr.FKinBody(M0e, Blist, thetalist)
            Tse = Tsb @ Tb0 @ T0e

            # feedback control (PI + feedforward, all in {e})
            V, Xerr, Xerr_int = FeedbackControl(
                Tse, Tse_d, Tse_d_next, Kp, Ki, dt, Xerr_int)

            # whole-robot Jacobian (6x9), u = [wheels(4), joints(5)]
            Je = whole_robot_body_jacobian(q, M0e, Blist, Tb0, r, l, w)
            u = np.linalg.pinv(Je, rcond=1e-4) @ V

            # clamp actuator speeds
            u[:4] = np.clip(u[:4],  -wheel_speed_max, wheel_speed_max)
            u[4:] = np.clip(u[4:],  -joint_speed_max, joint_speed_max)

            # integrate to next state
            # ensure your NextState signature matches (q, u, dt, ...)
            q = NextState(q, u, dt, wheel_speed_max)

            # order: phi, x, y, joint1..joint5, wheel1..wheel4, g
            row = [float(q[0]), float(q[1]), float(q[2])] \
                + [float(x) for x in q[3:8]] \
                + [float(x) for x in q[8:12]] \
                + [int(g)]
            writer.writerow(row)

            # Write the 13-column CSV the scene expects:

    print("Simulation complete. CSV written: youBot_Simulation.csv")


if __name__ == "__main__":
    main()

# %%
