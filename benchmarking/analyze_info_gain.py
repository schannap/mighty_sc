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

# Create plot
plt.figure(figsize=(10,10))
plt.xlabel("Time (s)")
plt.ylabel("Percentage of originally unknown voxels converted")
plt.title("Information gain with occlusion aware planner")
# Get the current axis
ax = plt.gca()

ax.xaxis.set_major_locator(ticker.MaxNLocator(6))
ax.yaxis.set_major_locator(ticker.MaxNLocator(6))

ax.xaxis.set_major_formatter(ticker.FormatStrFormatter('%.2f'))
ax.yaxis.set_major_formatter(ticker.FormatStrFormatter('%.2f'))

ax.tick_params(axis='both', labelsize=12)
# Loop through matching IDs
for traj_id in nominal_files.keys():
    if traj_id in occlusion_files:

        # Load CSVs
        df_nom = pd.read_csv(nominal_files[traj_id], skiprows=3)
        df_occ = pd.read_csv(occlusion_files[traj_id], skiprows=3)

        # Assumes columns are:
        # time_sec,percent_unknown (or similar)
        time_nom = np.array(df_nom.iloc[:, 0])
        perc_nom = np.array(df_nom.iloc[:, 1])

        time_occ = np.array(df_occ.iloc[:, 0])
        perc_occ = np.array(df_occ.iloc[:, 1])

        if len(time_nom) > len(time_occ):

            target_length = len(time_nom)
            padding_needed = target_length - len(perc_occ)

            # Pad arr1 with zeros at the end (after)
            # pad_width specifies padding for each axis: (before, after)
            padded_perc_occ = np.pad(perc_occ, (0, padding_needed), 'constant', constant_values=0)
            padded_perc_nom = perc_nom
        elif len(time_nom) <= len(time_occ):
            target_length = len(time_occ)
            padding_needed = target_length - len(perc_nom)

            # Pad arr1 with zeros at the end (after)
            # pad_width specifies padding for each axis: (before, after)
            padded_perc_nom = np.pad(perc_nom, (0, padding_needed), 'constant', constant_values=0)
            padded_perc_occ = perc_occ

        perc_diff = padded_perc_occ - padded_perc_nom
        
        # Plot
        plt.plot(time_nom, perc_nom, color='blue', alpha=0.4)
        plt.plot(time_occ, perc_occ, color='red', alpha=0.4)

print("Nominal files:", len(nominal_files))
print("Occlusion files:", len(occlusion_files))
print("Matching IDs:", set(nominal_files.keys()).intersection(occlusion_files.keys()))

# plt.grid(True)
plt.savefig("debug_plot.png", dpi=300)