import os
import re
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.ticker as ticker

# Directories
nominal_dir = '/home/intent/code/mighty_ws/src/mighty_sc/benchmark_data/csv_data'
occlusion_dir = '/home/intent/code/mighty_ws/src/mighty_sc/benchmark_data/csv_data_occ'


# Regex to extract 3 consecutive digits
pattern = re.compile(r'(\d{3})')

# Build dictionary: id -> filepath
nominal_files = {}
for f in os.listdir(nominal_dir):
    match = pattern.search(f)
    if match:
        nominal_files[match.group(1)] = os.path.join(nominal_dir, f)

occlusion_files = {}
for f in os.listdir(occlusion_dir):
    match = pattern.search(f)
    if match:
        occlusion_files[match.group(1)] = os.path.join(occlusion_dir, f)


max_length = 0


# First pass: determine max length
for traj_id in nominal_files.keys():
    if traj_id in occlusion_files:
        df_nom = pd.read_csv(nominal_files[traj_id], skiprows=3)
        df_occ = pd.read_csv(occlusion_files[traj_id], skiprows=3)

        max_length = max(max_length,
                         len(df_nom),
                         len(df_occ))


all_improvements = []

for traj_id in nominal_files.keys():
    if traj_id in occlusion_files:

        df_nom = pd.read_csv(nominal_files[traj_id], skiprows=3)
        df_occ = pd.read_csv(occlusion_files[traj_id], skiprows=3)

        perc_nom = np.array(df_nom.iloc[:, 1])
        perc_occ = np.array(df_occ.iloc[:, 1])

        # Pad with final value
        padded_nom = np.pad(
            perc_nom,
            (0, max_length - len(perc_nom)),
            mode='constant',
            constant_values=perc_nom[-1]
        )

        padded_occ = np.pad(
            perc_occ,
            (0, max_length - len(perc_occ)),
            mode='constant',
            constant_values=perc_occ[-1]
        )

        improvement = padded_occ - padded_nom
        all_improvements.append(improvement)

all_improvements = np.array(all_improvements)

mean_improvement = np.mean(all_improvements, axis=0)
std_improvement = np.std(all_improvements, axis=0)

plt.figure(figsize=(10,6))
plt.xlabel("Trajectory Index")
plt.ylabel("Improvement (Occlusion - Nominal)")
plt.title("Average Improvement (Final-Value Padded)")

ax = plt.gca()
ax.xaxis.set_major_locator(ticker.MaxNLocator(6))
ax.yaxis.set_major_locator(ticker.MaxNLocator(6))
ax.xaxis.set_major_formatter(ticker.FormatStrFormatter('%.2f'))
ax.yaxis.set_major_formatter(ticker.FormatStrFormatter('%.2f'))
ax.tick_params(axis='both', labelsize=12)

x = np.arange(max_length)

plt.plot(x, mean_improvement, linewidth=3)
plt.fill_between(x,
                 mean_improvement - std_improvement,
                 mean_improvement + std_improvement,
                 alpha=0.3)

plt.axhline(0, linestyle='--')
plt.tight_layout()
plt.savefig("average_improvement.png", dpi=300)