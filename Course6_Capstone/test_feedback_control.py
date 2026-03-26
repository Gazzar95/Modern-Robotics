# %% testing feedback control
#!/usr/bin/env python3
import numpy as np
import modern_robotics as mr
from FeedbackControl import FeedbackControl

def main():
    # 1) matrices from the MR wiki screenshot
    Xd = np.array([
        [0., 0., 1., 0.5],
        [0., 1., 0., 0.0],
        [-1., 0., 0., 0.5],
        [0., 0., 0., 1.]
    ])

    Xd_next = np.array([
        [0., 0., 1., 0.6],
        [0., 1., 0., 0.0],
        [-1., 0., 0., 0.3],
        [0., 0., 0., 1.]
    ])

    X = np.array([
        [0.170, 0.0,   0.985, 0.387],
        [0.0,   1.0,   0.0,   0.0],
        [-0.985, 0.0,  0.170, 0.570],
        [0.0,   0.0,   0.0,   1.0]
    ])

    dt = 0.01
    Kp = np.zeros((6, 6))
    Ki = np.zeros((6, 6))

    # ------------------------------------------------------------------
    # reproduce the wiki numbers manually
    # ------------------------------------------------------------------
    # feedforward twist Vd (in Xd frame)
    Vd_se3 = (1.0 / dt) * mr.MatrixLog6(np.dot(mr.TransInv(Xd), Xd_next))
    Vd = mr.se3ToVec(Vd_se3)       # should be (0, 0, 0, 20, 0, 10)

    # express in current end-effector frame: Ad_{X^{-1} Xd} Vd
    Ad = mr.Adjoint(np.dot(mr.TransInv(X), Xd))
    V_wiki = Ad @ Vd               # should be (0, 0, 0, 21.409, 0, 6.455)

    # configuration error X_err = log(X^{-1} Xd)
    Xerr_se3 = mr.MatrixLog6(np.dot(mr.TransInv(X), Xd))
    Xerr = mr.se3ToVec(Xerr_se3)   # should be (0, 0.171, 0, 0.080, 0, 0.107)

    Xerr_int_expected = Xerr * dt  # since integral starts at 0

    # ------------------------------------------------------------------
    # now call YOUR FeedbackControl and see if it produces the same
    # ------------------------------------------------------------------
    V_fc, Xerr_fc, Xerr_int_fc = FeedbackControl(
        X, Xd, Xd_next, Kp, Ki, dt
    )

    np.set_printoptions(precision=6, suppress=True)
    print("---- Wiki (manual) ----")
    print("Vd:", Vd)
    print("Ad * Vd:", V_wiki)
    print("Xerr:", Xerr)
    print("Xerr * dt:", Xerr_int_expected)

    print("\n---- Your FeedbackControl ----")
    print("V_fc:", V_fc)
    print("Xerr_fc:", Xerr_fc)
    print("Xerr_int_fc:", Xerr_int_fc)

    # quick numeric check
    tol = 1e-3
    def ok(a, b): return np.allclose(a, b, atol=tol, rtol=0)

    print("\nMatches wiki V?:", ok(V_fc, V_wiki))
    print("Matches wiki Xerr?:", ok(Xerr_fc, Xerr))
    print("Matches wiki Xerr_int?:", ok(Xerr_int_fc, Xerr_int_expected))

    if not ok(V_fc, V_wiki) or not ok(Xerr_fc, Xerr) or not ok(Xerr_int_fc, Xerr_int_expected):
        raise AssertionError("FeedbackControl output does not match MR wiki numbers within tolerance")

if __name__ == "__main__":
    main()


# %%
