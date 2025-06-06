import numpy as np
import modern_robotics as mr
import csv


def IKinBodyIterates(Blist, M, T, thetalist0, eomg, ev):
    """
    Numerically solves body-frame inverse kinematics via damped Newton-Raphson,
    logging each step and returning CSV & log filenames.

    Args:
      Blist     : 6xn NumPy array of body screw axes
      M         : 4x4 home configuration matrix
      T         : 4x4 desired end-effector configuration
      thetalist0: initial guess (length-n array)
      eomg      : angular error tolerance
      ev        : linear error tolerance

    Returns:
      (theta_history.csv, success_flag, ik_log.txt)
    """
    thetalist = np.array(thetalist0, dtype=float).copy()
    maxiters = 50
    alpha = 1.0       # overall scaling of the Newton step
    max_step = 0.2       # max radians per iteration
    log_lines = []
    theta_history = []

    success = False
    for i in range(maxiters):
        # 1) Current FK and error twist
        T_current = mr.FKinBody(M, Blist, thetalist)
        T_err = mr.TransInv(T_current) @ T
        Vb_mat = mr.MatrixLog6(T_err)
        Vb = mr.se3ToVec(Vb_mat)

        # 2) Compute error norms
        ang_err = np.linalg.norm(Vb[0:3])
        lin_err = np.linalg.norm(Vb[3:6])

        # 3) Log this iteration
        log_lines.append(f"Iteration {i+1}")
        log_lines.append(f"SE(3) current pose:\n{T_current}")
        log_lines.append(f"Error twist Vb: {Vb}")
        log_lines.append(f"Angular error ‖ω‖ = {ang_err:.6e}")
        log_lines.append(f"Linear  error ‖v‖ = {lin_err:.6e}\n")

        theta_history.append(thetalist.copy())

        # 4) Check convergence
        if ang_err < eomg and lin_err < ev:
            success = True
            break

        # 5) Damped Newton update
        Jb = mr.JacobianBody(Blist, thetalist)
        dtheta = alpha * (np.linalg.pinv(Jb) @ Vb)

        # 6) Clamp step magnitude
        step_norm = np.linalg.norm(dtheta)
        if step_norm > max_step:
            dtheta *= (max_step / step_norm)

        thetalist += dtheta

    # --- end loop ---

    # Write iteration log
    with open("ik_log.txt", "w", encoding="utf-8") as f:
        for line in log_lines:
            f.write(line + "\n")

    # Write theta history CSV
    with open("theta_history.csv", "w", newline="", encoding="utf-8") as csvfile:
        writer = csv.writer(csvfile)
        # header row
        n = len(thetalist0)
        writer.writerow([f"theta_{j+1}" for j in range(n)])
        # each row is one iteration's joint vector
        writer.writerows(theta_history)

    return "theta_history.csv", success, "ik_log.txt"
