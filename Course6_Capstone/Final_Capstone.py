# %%
# test milestone 2 of MR capstone
import numpy as np
import modern_robotics as mr
import csv
from Course6_Module2_PeerProj import TrajectoryGenerator

# test trajectory generator
if __name__ == "__main__":
    # Define some test configurations
    def rotz(theta):
        c, s = np.cos(theta), np.sin(theta)
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

    def T_from_xytheta(x, y, theta):
        T = np.eye(4)
        T[:3, :3] = rotz(theta)
        # cube frame is centered; floor -> cube center is 0.025 m (cube is 5 cm)
        T[:3, 3] = [x, y, 0.025]
        return T

    # --- From the wiki ---
    Tse_init = np.array([[0, 0, 1, 0],
                        [0, 1, 0, 0],
                        [-1, 0, 0, 0.5],
                        [0, 0, 0, 1]])  # reference EE initial pose (wiki)

    # cube initial (x=1, y=0, θ=0)
    Tsc_init = T_from_xytheta(1.0,  0.0, 0.0)
    # cube final   (x=0, y=-1, θ=-π/2)
    Tsc_final = T_from_xytheta(0.0, -1.0, -np.pi/2)

    # --- Your design choices (safe defaults for Scene 6) ---
    R_ce = np.array([[0, 0, 1],
                    [0, 1, 0],
                    [-1, 0, 0]])        # gripper z points down
    z_grasp = 0.02                  # meters above cube center for grasp
    z_standoff = 0.07                  # meters above cube center for standoff

    T_ce_grasp = np.eye(4)
    T_ce_grasp[:3, :3] = R_ce
    T_ce_grasp[:3, 3] = [0, 0, z_grasp]
    T_ce_standoff = np.eye(4)
    T_ce_standoff[:3, :3] = R_ce
    T_ce_standoff[:3, 3] = [0, 0, z_standoff]

    trajectory = TrajectoryGenerator(
        Tse_init, Tsc_init, Tsc_final, T_ce_grasp, T_ce_standoff)

    # Write trajectory to CSV
    with open('trajectory_output.csv', mode='w', newline='') as file:
        writer = csv.writer(file)
        # The TrajectoryGenerator in Course6_Module2_PeerProj returns an (N,13)
        # array where each row is [r11..r33, px, py, pz, gripper_state].
        # Write a 13-column header and handle both new (array rows) and
        # legacy (tuple of (Tse, is_hold, gripper_state)) formats.
        # No header written (remove headertest). The CSV will contain rows
        # matching the TrajectoryGenerator output: 13 values per row
        # -> [r11..r33, px, py, pz, gripper_state]

        for segment in trajectory:
            # If segment is a 13-element row (numpy array or list), write directly
            if hasattr(segment, '__len__') and len(segment) == 13:
                writer.writerow([float(x) for x in segment])
            else:
                # Legacy support: segment may be (Tse, is_hold, gripper_state)
                try:
                    Tse_segment, is_hold, gripper_state = segment
                    R = Tse_segment[:3, :3]
                    p = Tse_segment[:3, 3]
                    row = list(R.reshape(9)) + list(p) + [gripper_state]
                    writer.writerow([float(x) for x in row])
                except Exception:
                    # Fallback: write string representation
                    writer.writerow([str(segment)])

    print("Trajectory written to trajectory_output.csv")
# %%
