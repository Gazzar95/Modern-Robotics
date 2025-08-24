# %%Cource 5 Module 1 Peer Project
# givens: xi, yi, n_xi, n_yi
# output: Form Closure: True or False
import numpy as np
from numpy.linalg import matrix_rank
from scipy.optimize import linprog


def wrench_from_contact(x, y, nx, ny):
    """Compute planar wrench"""
    tau = x*ny - y*nx
    return np.array([tau, nx, ny])


def build_F(contacts):
    """Build wrench matrix F (3 x m) from contact list [(x,y,nx,ny), ...]."""
    cols = []
    for (x, y, nx, ny) in contacts:
        n = np.hypot(nx, ny)
        nx, ny = nx/n, ny/n  # normalize normal
        cols.append(wrench_from_contact(x, y, nx, ny))
    return np.column_stack(cols)


def is_form_closure(F):
    """Check form closure"""
    if matrix_rank(F) < 3:
        return False, None

    m = F.shape[1]
    c = np.zeros(m + 1)
    c[-1] = -1.0  # minimize -t

    # Equalities: F λ = 0,  1^T λ = 1
    A_eq = np.zeros((F.shape[0] + 1, m + 1))
    A_eq[:F.shape[0], :m] = F
    A_eq[-1, :m] = 1.0
    b_eq = np.zeros(F.shape[0] + 1)
    b_eq[-1] = 1.0

    # Inequalities: -λ + t*1 <= 0
    A_ub = np.zeros((m, m + 1))
    A_ub[:, :m] = -np.eye(m)
    A_ub[:, -1] = 1.0
    b_ub = np.zeros(m)

    bounds = [(0, None)] * m + [(0, None)]

    res = linprog(c, A_ub=A_ub, b_ub=b_ub, A_eq=A_eq, b_eq=b_eq,
                  bounds=bounds, method="highs")

    if not res.success:
        return False, None

    lam = res.x[:m]
    t_star = res.x[-1]
    return (t_star > 1e-8), lam


if __name__ == "__main__":
    print("\033[4mForm closure Example 12.7 from MR\033[0m")
    contacts_fc = [
        (0.0, 0.0, -1.0,  0.0),
        (0.0, 0.0,  0.0, -1.0),
        (2.0, 1.0,  1.0,  0.0),
        (2.0, 1.0,  0.0,  1.0),
    ]
    F_fc = build_F(contacts_fc)
    is_fc, k = is_form_closure(F_fc)
    if is_fc:
        print("Form closure")
        print("k:", k)
    else:
        print("Not form closure")
        if k is not None:
            print("Candidate k:", k)

    print("\n\033[4mNot Form closure Example 12.7 from MR\033[0m")
    contacts_nfc = [
        (0.0, 0.0, -1.0,  0.0),
        (0.0, 0.0,  0.0, -1.0),
        (2.0, 0.0,  1.0,  0.0),
        (2.0, 0.0,  0.0,  -1.0),
    ]
    F_nfc = build_F(contacts_nfc)
    is_fc, k = is_form_closure(F_nfc)
    if is_fc:
        print("Form closure")
        print("k:", k)
    else:
        print("Not form closure")
        if k is not None:
            print("Candidate k:", k)

# %%
