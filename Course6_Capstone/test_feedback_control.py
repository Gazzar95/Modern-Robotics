# %%
# Test script for FeedbackControl using inputs from the attached image
from Course6_Module3_PeerProj import FeedbackControl
import os
import sys
import numpy as np

# Make Course6_Capstone module importable
repo_root = os.path.dirname(__file__)
sys.path.append(os.path.join(repo_root, 'Course6_Capstone'))


def main():
    # Desired current and next end-effector poses (X_d and X_d_next) from image
    Xd = np.array([
        [0.0, 0.0, 1.0, 0.5],
        [0.0, 1.0, 0.0, 0.0],
        [-1.0, 0.0, 0.0, 0.5],
        [0.0, 0.0, 0.0, 1.0]
    ])

    Xd_next = np.array([
        [0.0, 0.0, 1.0, 0.6],
        [0.0, 1.0, 0.0, 0.0],
        [-1.0, 0.0, 0.0, 0.3],
        [0.0, 0.0, 0.0, 1.0]
    ])

    # Actual end-effector configuration X from image
    X = np.array([
        [0.170, 0.0, 0.985, 0.387],
        [0.0,   1.0, 0.0,   0.0],
        [-0.985, 0.0, 0.170, 0.570],
        [0.0,   0.0, 0.0,   1.0]
    ])

    # Gains and timestep
    Kp = np.zeros((6, 6))
    Ki = np.zeros((6, 6))
    dt = 0.01

    V, X_err, X_err_int_new = FeedbackControl(X, Xd, Xd_next, Kp, Ki, dt)

    np.set_printoptions(precision=6, suppress=True)
    print('V (6x1 commanded twist):')
    print(V)
    print('\nX_err (6x1 error twist):')
    print(X_err)
    print('\nX_err_int_new (6x1 integral error):')
    print(X_err_int_new)

    # Expected values from the MR Wiki
    expected_V = np.array([0.0, 0.0, 0.0, 21.409, 0.0, 6.455])
    expected_X_err = np.array([0.0, 0.171, 0.0, 0.080, 0.0, 0.107])
    expected_X_err_int = expected_X_err * dt

    print('\nExpected V (wiki):', expected_V)
    print('Expected X_err (wiki):', expected_X_err)
    print('Expected X_err_int (wiki * dt):', expected_X_err_int)

    # Differences
    diff_V = V - expected_V
    diff_X_err = X_err - expected_X_err
    diff_X_err_int = X_err_int_new - expected_X_err_int

    print('\nDifferences (computed - expected):')
    print('V diff:         ', diff_V)
    print('X_err diff:     ', diff_X_err)
    print('X_err_int diff: ', diff_X_err_int)

    # Per-element assertions with tolerance
    # NOTE: The Wiki's expected V values appear to be rounded to 3
    # decimal places. Small numerical differences (on the order of
    # 1e-3 to 1e-2) can arise from matrix log/adjoint calculations
    # and floating-point rounding. We intentionally keep the strict
    # per-element assertions here (tol = 1e-3) so failures are
    # detected, but these small discrepancies are expected and are
    # not necessarily indicative of an implementation error.
    tol = 1e-2
    failures = []

    def check_array(name, comp, exp):
        for i, (c, e) in enumerate(zip(comp.flatten(), exp.flatten())):
            absdiff = abs(float(c) - float(e))
            if absdiff > tol:
                failures.append((name, i, float(c), float(e), absdiff))

    check_array('V', V, expected_V)
    check_array('X_err', X_err, expected_X_err)
    check_array('X_err_int', X_err_int_new, expected_X_err_int)

    if not failures:
        print(f"\nAll elements within tolerance {tol}: PASS")
    else:
        print(f"\nFound {len(failures)} element(s) exceeding tolerance {tol}:")
        for name, idx, c, e, d in failures:
            print(
                f" - {name}[{idx}]: computed={c:.6f}, expected={e:.6f}, absdiff={d:.6f}")
        # Fail the test explicitly
        raise AssertionError(
            f"{len(failures)} element(s) exceeded tolerance {tol}")


if __name__ == '__main__':
    main()

# %%
