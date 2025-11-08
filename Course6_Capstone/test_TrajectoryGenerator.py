# %%
# testing trajectory generator

import numpy as np
import csv
from TrajectoryGenerator import TrajectoryGenerator


def main():
    # Same poses you were using in your main script earlier
    # Trajectory Variables
    Tse_init = np.array([
        [0, 0, 1, 0.5],
        [0, 1, 0, 0.0],
        [-1, 0, 0, 0.5],
        [0, 0, 0, 1]
    ])

    Tsc_init = np.array([
        [1, 0, 0, 1.0],
        [0, 1, 0, 0.0],
        [0, 0, 1, 0.025],
        [0, 0, 0, 1]
    ])

    Tsc_final = np.array([
        [0, 1, 0, 0.0],
        [-1, 0, 0, -1.0],
        [0, 0, 1, 0.025],
        [0, 0, 0, 1]
    ])

    Tce_grasp = np.array([
        [0, 0, 1, 0.0],
        [0, 1, 0, 0.0],
        [-1, 0, 0, 0.015],
        [0, 0, 0, 1]
    ])

    Tce_standoff = np.array([
        [0, 0, 1, 0.0],
        [0, 1, 0, 0.0],
        [-1, 0, 0, 0.05],
        [0, 0, 0, 1]
    ])

    # ----- Timing -----
    dt = 0.01

    # ---- call your trajectory generator ----
    traj = TrajectoryGenerator(
        Tse_init, Tsc_init, Tsc_final, Tce_grasp, Tce_standoff, dt=dt
    )
    # At this point, in the MR capstone, traj is typically an N x 13 array/list

    # ---- write to CSV in the exact order CoppeliaSim wants ----
    out_fname = "trajectory.csv"
    with open(out_fname, mode="w", newline="") as f:
        writer = csv.writer(f)
        for row in traj:
            # row should already be: [r11,...,r33, px, py, pz, gripper]
            writer.writerow(row)

    print(f"Wrote {len(traj)} rows to {out_fname}")


if __name__ == "__main__":
    main()
# %%
