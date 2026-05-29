import os
import re
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# =========================
# USER CONFIG
# =========================
nominal_dir = '/home/intent/code/mighty_ws/src/mighty_sc/benchmark_data/18_nominal_filtered_vis'
occlusion_dir = '/home/intent/code/mighty_ws/src/mighty_sc/benchmark_data/18_filtered_vis'

# =========================
# Regex to match trajectory IDs
# =========================
pattern = re.compile(r'(\d{3})')

def load_files(directory):
    files = {}
    for f in os.listdir(directory):
        match = pattern.search(f)
        if match:
            files[match.group(1)] = os.path.join(directory, f)
    return files

nominal_files = load_files(nominal_dir)
occlusion_files = load_files(occlusion_dir)

# =========================
# Sort trajectory IDs
# =========================
common_ids = sorted(set(nominal_files.keys()) & set(occlusion_files.keys()))

print(f"Found {len(common_ids)} matching trajectory pairs.")

# =========================
# Loop and plot
# =========================
for traj_id in common_ids:

    print(f"\nShowing trajectory {traj_id}")

    df_nom = pd.read_csv(nominal_files[traj_id], skiprows=3)
    df_occ = pd.read_csv(occlusion_files[traj_id], skiprows=3)

    perc_nom = np.array(df_nom.iloc[:, 1])
    perc_occ = np.array(df_occ.iloc[:, 1])

    t_nom = np.arange(len(perc_nom))
    t_occ = np.arange(len(perc_occ))

    plt.figure(figsize=(8,5))

    plt.plot(t_nom, perc_nom, label="Nominal", linewidth=2)
    plt.plot(t_occ, perc_occ, label="Occlusion-aware", linewidth=2)

    plt.xlabel("Time Step")
    plt.ylabel("Visibility (%)")
    plt.title(f"Trajectory {traj_id}: Visibility vs Time")

    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()

print("\nDone.")