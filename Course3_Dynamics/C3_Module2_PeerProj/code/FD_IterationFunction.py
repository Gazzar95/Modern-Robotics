import numpy as np
from modern_robotics import ForwardDynamics, EulerStep



def ForwardDynamicsIterate(thetalist, dthetalist, taumat, g, Ftipmat, \
                              Mlist, Glist, Slist, dt, intRes):
    """
    Simulates the motion of a serial chain given an open-loop history of
    joint forces/torques

    :param thetalist: n-vector of initial joint variables
    :param dthetalist: n-vector of initial joint rates
    :param taumat: An N x n matrix of joint forces/torques, where each row is
                   the joint effort at any time step
    :param g: Gravity vector g
    :param Ftipmat: An N x 6 matrix of spatial forces applied by the end-
                    effector (If there are no tip forces the user should
                    input a zero and a zero matrix will be used)
    :param Mlist: List of link frames {i} relative to {i-1} at the home
                  position
    :param Glist: Spatial inertia matrices Gi of the links
    :param Slist: Screw axes Si of the joints in a space frame, in the format
                  of a matrix with axes as the columns
    :param dt: The timestep between consecutive joint forces/torques
    :param intRes: Integration resolution is the number of times integration
                   (Euler) takes places between each time step. Must be an
                   integer value greater than or equal to 1
    :return thetamat: The N x n matrix of robot joint angles resulting from
                      the specified joint forces/torques
    :return dthetamat: The N x n matrix of robot joint velocities
    This function calls a numerical integration procedure that uses
    ForwardDynamics.
    """


    taumat = np.array(taumat).T
    Ftipmat = np.array(Ftipmat).T
    thetamat = taumat.copy().astype(float)
    thetamat[:, 0] = thetalist
    dthetamat = taumat.copy().astype(float)
    dthetamat[:, 0] = dthetalist

    for i in range(np.array(taumat).shape[1] - 1):
        for j in range(intRes):
            ddthetalist \
            = ForwardDynamics(thetalist, dthetalist, taumat[:, i], g, \
                              Ftipmat[:, i], Mlist, Glist, Slist)
            thetalist,dthetalist = EulerStep(thetalist, dthetalist, \
                                             ddthetalist, 1.0 * dt / intRes)
        thetamat[:, i + 1] = thetalist
        dthetamat[:, i + 1] = dthetalist
    thetamat = np.array(thetamat).T
    dthetamat = np.array(dthetamat).T

    np.savetxt("theta_history.csv", thetamat, delimiter=",")

    return "theta_history.csv"

