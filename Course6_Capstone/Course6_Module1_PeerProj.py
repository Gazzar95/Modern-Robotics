# Course 6: Capstone Project
# Milestone 1: youBot Kinematics Simulator & CSV output
# Input:
# 1) 12 vector of current config
#       - content - [chassis phi, chassis x, chassis y, J1, J2, J3, J4, J5, W1, W2, W3, W4]
# 2) 9 vector of joint and wheel speeds
#       - content - [W1d, W2d, W3d, W4d, J1d, J2d, J3d, J4d, J5d]
# 3) timestep dt
# 4) Max and Min joint angles   <-- (used here as a single max speed clamp)
# Output:
# 1) 12 vector of next config at time t+dt

import numpy as np
import modern_robotics as mr
import csv


def NextState(current_config, joint_speeds, dt, joint_limits):
    # --------- unpack state ----------
    # current_config is length-12 1D: [phi, x, y, J1..J5, W1..W4]
    phi, x, y = current_config[0:3]
    arm_position = current_config[3:8]    # rad (5)
    wheel_position = current_config[8:12]   # rad (4)

    # --------- unpack controls ----------
    # joint_speeds is length-9 1D: [W1d..W4d, J1d..J5d]
    wheel_speeds = joint_speeds[0:4]        # rad/s
    arm_speeds = joint_speeds[4:9]        # rad/s

    # --------- clamp speeds (spec allows one limit) ----------
    maxspd = float(joint_limits)
    wheel_speeds = np.clip(wheel_speeds, -maxspd, maxspd)
    arm_speeds = np.clip(arm_speeds,   -maxspd, maxspd)

    # --------- Arm/Wheel angle w/ Euler Step ----------
    arm_position_new = arm_position + arm_speeds * dt
    wheel_position_new = wheel_position + wheel_speeds * dt

    # --------- Chassis position w/ H(0) mapping (MR 13.10) ----------
    # u = (1/r) * H0 * Vb  ==>  Vb_increment = F * dphi_wheels
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
    dphi_wheels = wheel_speeds * dt        # wheel angle increments
    dqb_body = F @ dphi_wheels             # [dphi, dx_b, dy_b] over dt

    dphi_b, dx_b, dy_b = dqb_body
    # rotate body-frame translation to world using current heading
    c, s = np.cos(phi), np.sin(phi)
    dx_w = c * dx_b - s * dy_b
    dy_w = s * dx_b + c * dy_b

    phi_new = phi + dphi_b
    x_new = x + dx_w
    y_new = y + dy_w

    # --------- pack next config ----------
    NextConfig = np.concatenate((
        np.array([phi_new, x_new, y_new]),
        arm_position_new,
        wheel_position_new
    ))
    return NextConfig


# --------- simple CSV driver (example) ----------
if __name__ == "__main__":
    # initial config (all zeros)
    q = np.zeros(12)
    # example constant controls: [W1d..W4d, J1d..J5d]
    u = np.array([0.5, 0.5, 0.5, 0.5,  0.1, 0.0, -0.1, 0.0, 0.05])
    dt = 0.01
    steps = 100
    max_speed = 12.3

    traj = [q.copy()]
    for _ in range(steps):
        q = NextState(q, u, dt, max_speed)
        traj.append(q.copy())

    header = ["chassis phi", "chassis x", "chassis y", "J1",
              "J2", "J3", "J4", "J5", "W1", "W2", "W3", "W4"]
    with open("milestone1_trajectory.csv", "w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(header)
        wr.writerows(traj)

    print("Wrote milestone1_trajectory.csv")
