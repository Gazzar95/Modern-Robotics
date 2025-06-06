# %% C3 Module 1 Assignment 
# Problem 1
import numpy as np

p  = 5600          # kg/m³
D   = 0.04          # m  – handle diameter
L   = 0.20          # m  – handle length
D_s = 0.20          # m  – sphere (head) diameter

r_c = D   / 2       # cylinder radius
r_s = D_s / 2       # sphere radius

# masses
M_c = np.pi * r_c**2 * L * p
M_s = (4/3) * np.pi * r_s**3 * p

# distance from dumbbell C‑of‑M to each sphere centre
d   = L/2 + r_s

# --- sphere contributions (two identical spheres) ---
I_cm_s  = (2/5) * M_s * r_s**2          # inertia at its own COM (any axis)
Ix_sph  = I_cm_s                        # axis collinear with rod – no offset
Iy_sph  = I_cm_s + M_s * d**2           # axis ⟂ rod – use d
Iz_sph  = Iy_sph                        # symmetry

# multiply by 2 spheres
Ix_sph *= 2
Iy_sph *= 2
Iz_sph *= 2

# --- rod (solid cylinder, COM already at origin) ---
Ix_cyl = (1/2) * M_c * r_c**2
Iy_cyl = (1/12) * M_c * (3*r_c**2 + L**2)
Iz_cyl = Iy_cyl

# --- total inertia tensor about dumbbell COM ---
Ix = Ix_sph + Ix_cyl
Iy = Iy_sph + Iy_cyl
Iz = Iz_sph + Iz_cyl

I = np.diag([Ix, Iy, Iz])
print(I.tolist())

# %% Problem 5
import modern_robotics as mr
import numpy as np

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

thetalist = [0, np.pi/6, np.pi/4, np.pi/3, np.pi/2, 2*np.pi/3]
dthetalist = [0.2, 0.2, 0.2, 0.2, 0.2, 0.2]
ddthetalist = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
g = [0, 0, -9.81]  # gravity vector
Ftip = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]  # external force at the end-effector

taulist = mr.InverseDynamics(thetalist, dthetalist, ddthetalist, g, Ftip, Mlist, Glist, Slist)
print("Torque list:", taulist.tolist())



# %%
