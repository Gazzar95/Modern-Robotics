# %% Course 5: Manipulation and Mobile Robotics
# %% Module 2: Peer-graded Project: Mobile Robot Programming
# Goal: Write code to determine if an assembly of rigid bodies in friction contact with eah other can remain standing in gravity.
# inputs:
# 1) Dis of static mass properties of each N bodies. Includes:(x, y) locations and center of mass
# 2) Dis of contact. Includes: contact ID, contact location, contact normal direction, friction coefficient at contact.
# outputs:
# 1) Whether the assembly is stable or not.

import numpy as np
from scipy.optimize import linprog

# ---------- geometry helpers ----------


def edge_dirs(nx, ny, mu):
    """Unit edge directions of 2D friction cone from unit normal (nx,ny)."""
    n = np.hypot(nx, ny)
    nx, ny = nx/n, ny/n
    tx, ty = -ny, nx  # +90° rotation
    th = np.arctan(mu)
    c, s = np.cos(th), np.sin(th)
    ux_p, uy_p = c*nx + s*tx, c*ny + s*ty
    ux_m, uy_m = c*nx - s*tx, c*ny - s*ty
    return (ux_p, uy_p), (ux_m, uy_m)


def wrench_from_edge(x, y, ux, uy, sign=+1):
    """Wrench [Fx, Fy, Tau] about origin from a unit edge direction at (x,y)."""
    fx, fy = sign*ux, sign*uy
    tau = x*fy - y*fx
    return np.array([fx, fy, tau])

# ---------- input data parsing ----------


def from_project_spec(raw_contacts, angle_in_degrees=False):
    """
    raw_contacts: list of (a, b, x, y, angle, mu)
    Returns:
      contacts: [(x,y,nx,ny), ...]
      mus:      [mu_i, ...]
      pairs:    [(a_idx, b_idx), ...]  # a_idx/b_idx in 0..M-1 for free bodies; None for ground
    """
    contacts, mus, pairs = [], [], []
    for (a, b, x, y, ang, mu) in raw_contacts:
        if angle_in_degrees:
            ang = np.deg2rad(ang)
        nx, ny = np.cos(ang), np.sin(ang)  # normal into bodyA
        contacts.append((x, y, nx, ny))
        mus.append(mu)
        a_idx = a-1 if a > 0 else None
        b_idx = b-1 if b > 0 else None
        pairs.append((a_idx, b_idx))
    return contacts, mus, pairs

# ---------- assembly of equilibrium ----------


def assemble_F_b(contacts, mus, pairs, masses, coms, g=9.81):
    """
    contacts: [(x,y,nx,ny), ...]
    mus:      [mu_i, ...]
    pairs:    [(a_idx,b_idx)] with None = ground
    masses:   [m_j] for free bodies j=0..M-1  (project bodies 1..M)
    coms:     [(xc_j, yc_j)] per free body
    Returns F (3M x 2N), b (3M,)
    """
    M, N = len(masses), len(contacts)
    F = np.zeros((3*M, 2*N))
    b = np.zeros(3*M)

    # RHS from gravity (use CGs)
    for j, (m, (xc, yc)) in enumerate(zip(masses, coms)):
        fx, fy = 0.0, -m*g
        tau = xc*fy - yc*fx
        b[3*j:3*j+3] = -np.array([fx, fy, tau])

    # Columns: two edges per contact
    for i, ((x, y, nx, ny), mu, (a, bdy)) in enumerate(zip(contacts, mus, pairs)):
        (uxp, uyp), (uxm, uym) = edge_dirs(nx, ny, mu)
        cP, cM = 2*i, 2*i+1

        if a is not None:
            F[3*a:3*a+3, cP] = wrench_from_edge(x, y, uxp, uyp, +1)
            F[3*a:3*a+3, cM] = wrench_from_edge(x, y, uxm, uym, +1)
        if bdy is not None:
            F[3*bdy:3*bdy+3, cP] = wrench_from_edge(x, y, uxp, uyp, -1)
            F[3*bdy:3*bdy+3, cM] = wrench_from_edge(x, y, uxm, uym, -1)

    return F, b

# ---------- Linprog check ----------


def is_stable(F, b, tol=1e-8):
    """Solve F k = b, k >= 0; returns (feasible: bool, k or None)."""
    m = F.shape[1]
    res = linprog(c=np.ones(m), A_eq=F, b_eq=b,
                  bounds=[(0, None)]*m, method="highs")
    if not res.success:
        return False, None
    if np.linalg.norm(F @ res.x - b, np.inf) > tol:
        return False, None
    k = np.clip(res.x, 0, None)
    return True, k


# ---------- example wiring (no answers printed) ----------
if __name__ == "__main__":
    # 1) Define raw_contacts per project spec (fill with your case):
    raw_contacts = [
        # (a, b, x, y, normal_angle, mu)
        # e.g., (1, 2, 60, 60, np.pi, 0.5)
        # e.g., (2, 0, 72, 0, 1.5708, 0.25)
    ]

    # 2) Bodies 1..M: masses and COMs (world coords)
    masses = []            # e.g., [m1, m2, ...]
    coms = []            # e.g., [(xc1, yc1), (xc2, yc2), ...]

    # Parse & assemble
    contacts, mus, pairs = from_project_spec(
        raw_contacts, angle_in_degrees=False)
    F, b = assemble_F_b(contacts, mus, pairs, masses, coms, g=9.81)

    # Now call: feasible, k = is_stable(F, b)
    # (leave it to you to print/inspect as you wish)
