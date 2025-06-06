# %% Peer Graded Assignment C2 Module 3
# Goal: modify IKinbody function to report imtermediate iterates of the Newton-Raphson
# Each reported iteration must include:
# 1) i = iteration number
# 2) Theta = Joint vector
# 3) T = End effector configuration
# 4) V = Error twist
# 5) w = Angular error
# 6) v = Linear error

import numpy as np
import modern_robotics as mr

# --------------------------MR Lib Code Section -----------------------------------------


def IKinBodyIterates(Blist, M, T, thetalist0, eomg, ev):

    thetalist = np.array(thetalist0).copy()
    i = 0

    maxiterations = 60
    Vb = mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(mr.FKinBody(M, Blist,
                                                      thetalist)), T)))
    err = np.linalg.norm([Vb[0], Vb[1], Vb[2]]) > eomg \
        or np.linalg.norm([Vb[3], Vb[4], Vb[5]]) > ev
    log_lines = []  # adding empty list
    theta_history = []  # adding empty list
    while err and i < maxiterations:
        thetalist = thetalist \
            + np.dot(np.linalg.pinv(mr.JacobianBody(Blist,
                                                    thetalist)), Vb)
        i = i + 1
        Vb \
            = mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(mr.FKinBody(M, Blist,
                                                           thetalist)), T)))
        err = np.linalg.norm([Vb[0], Vb[1], Vb[2]]) > eomg \
            or np.linalg.norm([Vb[3], Vb[4], Vb[5]]) > ev

        # End offector config for each iteration
        T_theta = mr.FKinBody(M, Blist, thetalist)

        # logging iteration results
        log_lines.append(f"Iteration: {i}")
        log_lines.append(f"SE(3) end - effector config: {T_theta}")
        log_lines.append(f"error twist V_b: {Vb}")
        log_lines.append(
            f"Angular error magnitude ||omega_b||: {np.linalg.norm([Vb[0], Vb[1], Vb[2]])}")
        log_lines.append(
            f"Linear error magnitude ||v_b||: {np.linalg.norm([Vb[3], Vb[4], Vb[5]])} \n")

        # Log theta history as .cvs
        theta_history.append(thetalist)

    # Save iteration results as .txt
    f = open("ik_log.txt", "w")
    for line in log_lines:
        f.write(line + "\n")
    f.close()

    # Save theta history as .cvs
    import csv
    f = open("theta_history.csv", "w")
    f.write("theta_1, theta_2, theta_3, theta_4\n")
    for row in theta_history:
        line = ",".join(str(value) for value in row)
        f.write(line + "\n")
    f.close()

    return ("theta_history.csv", not err, "ik_log.txt")

# %%
