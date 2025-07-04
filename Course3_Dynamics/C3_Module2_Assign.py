# %% Course 3 Module 2 Assignment
# Problem 1: robot 
import numpy as np
import modern_robotics as mr

M01 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.089159], [0, 0, 0, 1]]
M12 = [[0, 0, 1, 0.28], [0, 1, 0, 0.13585], [-1, 0, 0, 0], [0, 0, 0, 1]]
M23 = [[1, 0, 0, 0], [0, 1, 0, -0.1197], [0, 0, 1, 0.395], [0, 0, 0, 1]]
M34 = [[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0.14225], [0, 0, 0, 1]]
M45 = [[1, 0, 0, 0], [0, 1, 0, 0.093], [0, 0, 1, 0], [0, 0, 0, 1]]
M56 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.09465], [0, 0, 0, 1]]
M67 = [[1, 0, 0, 0], [0, 0, 1, 0.0823], [0, -1, 0, 0], [0, 0, 0, 1]]
G1 = np.diag([0.010267495893, 0.010267495893,  0.00666, 3.7, 3.7, 3.7])
G2 = np.diag([0.22689067591, 0.22689067591, 0.0151074, 8.393, 8.393, 8.393])
G3 = np.diag([0.049443313556, 0.049443313556, 0.004095, 2.275, 2.275, 2.275])
G4 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
G5 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
G6 = np.diag([0.0171364731454, 0.0171364731454, 0.033822, 0.1879, 0.1879, 0.1879])
Glist = [G1, G2, G3, G4, G5, G6]
Mlist = [M01, M12, M23, M34, M45, M56, M67] 
Slist = [[0,         0,         0,         0,        0,        0],
         [0,         1,         1,         1,        0,        1],
         [1,         0,         0,         0,       -1,        0],
         [0, -0.089159, -0.089159, -0.089159, -0.10915, 0.005491],
         [0,         0,         0,         0,  0.81725,        0],
         [0,         0,     0.425,   0.81725,        0,  0.81725]]

thetalist = np.array([0, np.pi/6, np.pi/4, np.pi/3, np.pi/2, 2*np.pi/3]) # Joint angles
dthetalist = np.array([0.2, 0.2, 0.2, 0.2, 0.2, 0.2]) #Velocity
ddthetalist = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1]) #Acceleration
g = np.array([0, 0, -9.81])  # Gravity vector
Ftip = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])  # External force at the end-effector
taulist = np.array([0.0128, -41.1477, -3.7809, 00.0323, 0.0370, 0.1034])  # Joint torques


M = mr.MassMatrix(thetalist, Mlist, Glist, Slist)
print("Mass Matrix M:")
print(M.tolist())

c = mr.VelQuadraticForces(thetalist, dthetalist, Mlist, Glist, Slist)
print("Coriolis and centrifugal forces c:")
print(c.tolist())

grav = mr.GravityForces(thetalist, g, Mlist, Glist, Slist)
print("Gravitational forces grav:")
print(grav.tolist())

JTFtip = mr.EndEffectorForces(thetalist, Ftip, Mlist, Glist, Slist)
print("End-effector forces JTFtip:")
print(JTFtip.tolist())

ddthetalist = mr.ForwardDynamics(thetalist, dthetalist, taulist, g, Ftip, Mlist, Glist, Slist)
print("Joint accelerations ddthetalist:")
print(ddthetalist.tolist())

# %%
