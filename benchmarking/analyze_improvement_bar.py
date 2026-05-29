# import os
# import re
# import pandas as pd
# import matplotlib.pyplot as plt
# import numpy as np
# import matplotlib.ticker as ticker

# # =========================
# # Directories
# # =========================
# nominal_dir = '/home/intent/code/mighty_ws/src/mighty_sc/benchmark_data/18_nominal_vis'
# occlusion_dir = '/home/intent/code/mighty_ws/src/mighty_sc/benchmark_data/18_vis'

# # =========================
# # Regex to match trajectory IDs
# # =========================
# pattern = re.compile(r'(\d{3})')

# nominal_files = {}
# for f in os.listdir(nominal_dir):
#     match = pattern.search(f)
#     if match:
#         nominal_files[match.group(1)] = os.path.join(nominal_dir, f)

# occlusion_files = {}
# for f in os.listdir(occlusion_dir):
#     match = pattern.search(f)
#     if match:
#         occlusion_files[match.group(1)] = os.path.join(occlusion_dir, f)

# # =========================
# # Storage
# # =========================
# all_improvements = []

# auc_diffs = []

# time_to_50_nom = []
# time_to_50_occ = []

# max_length = 0

# # =========================
# # First pass: determine max trajectory length
# # =========================
# for traj_id in nominal_files:
#     if traj_id in occlusion_files:

#         df_nom = pd.read_csv(nominal_files[traj_id], skiprows=3)
#         df_occ = pd.read_csv(occlusion_files[traj_id], skiprows=3)

#         max_length = max(max_length, len(df_nom), len(df_occ))

# # =========================
# # Helper: time to reach target visibility
# # =========================
# def time_to_visibility(arr, threshold):
#     for i, val in enumerate(arr):
#         if val >= threshold:
#             return i
#     return len(arr)

# # =========================
# # Main processing loop
# # =========================
# for traj_id in nominal_files:

#     if traj_id not in occlusion_files:
#         continue

#     df_nom = pd.read_csv(nominal_files[traj_id], skiprows=3)
#     df_occ = pd.read_csv(occlusion_files[traj_id], skiprows=3)

#     perc_nom = np.array(df_nom.iloc[:, 1])
#     perc_occ = np.array(df_occ.iloc[:, 1])

#     # =========================
#     # AUC metric (information gained sooner)
#     # =========================
#     auc_nom = np.trapz(perc_nom) / len(perc_nom)
#     auc_occ = np.trapz(perc_occ) / len(perc_occ)

#     auc_diffs.append(auc_occ - auc_nom)

#     # =========================
#     # Time to reach 50% visibility
#     # =========================
#     time_to_50_nom.append(time_to_visibility(perc_nom, 50))
#     time_to_50_occ.append(time_to_visibility(perc_occ, 50))

#     # =========================
#     # Pad curves for mean improvement plot
#     # =========================
#     padded_nom = np.pad(
#         perc_nom,
#         (0, max_length - len(perc_nom)),
#         mode='edge'
#     )

#     padded_occ = np.pad(
#         perc_occ,
#         (0, max_length - len(perc_occ)),
#         mode='edge'
#     )

#     improvement = padded_occ - padded_nom
#     all_improvements.append(improvement)

# # =========================
# # Convert to numpy
# # =========================
# all_improvements = np.array(all_improvements)
# auc_diffs = np.array(auc_diffs)

# time_to_50_nom = np.array(time_to_50_nom)
# time_to_50_occ = np.array(time_to_50_occ)

# # =========================
# # Statistics
# # =========================
# print("\n===== AUC IMPROVEMENT =====")
# print("Mean:", np.mean(auc_diffs))
# print("Std:", np.std(auc_diffs))
# print("Max:", np.max(auc_diffs))
# print("Min:", np.min(auc_diffs))

# print("\n===== TIME TO 50% VISIBILITY =====")
# print("Nominal mean:", np.mean(time_to_50_nom))
# print("Occlusion mean:", np.mean(time_to_50_occ))
# print("Mean improvement:", np.mean(time_to_50_nom - time_to_50_occ))

# # =========================
# # Improvement curve
# # =========================
# mean_improvement = np.mean(all_improvements, axis=0)
# std_improvement = np.std(all_improvements, axis=0)

# plt.figure(figsize=(10,6))
# plt.xlabel("Trajectory Index")
# plt.ylabel("Visibility Improvement (Occlusion - Nominal)")
# plt.title("Average Visibility Improvement Over Time")

# ax = plt.gca()
# ax.xaxis.set_major_locator(ticker.MaxNLocator(6))
# ax.yaxis.set_major_locator(ticker.MaxNLocator(6))
# ax.xaxis.set_major_formatter(ticker.FormatStrFormatter('%.2f'))
# ax.yaxis.set_major_formatter(ticker.FormatStrFormatter('%.2f'))
# ax.tick_params(axis='both', labelsize=12)

# x = np.arange(max_length)

# plt.plot(x, mean_improvement, linewidth=3)

# plt.fill_between(
#     x,
#     mean_improvement - std_improvement,
#     mean_improvement + std_improvement,
#     alpha=0.3
# )

# plt.axhline(0, linestyle='--')

# plt.tight_layout()
# plt.savefig("average_visibility_improvement_18.png", dpi=300)

# # =========================
# # AUC histogram
# # =========================
# plt.figure(figsize=(8,5))

# plt.hist(auc_diffs, bins=20)
# plt.axvline(0, linestyle='--')

# plt.xlabel("AUC Improvement (Occlusion - Nominal)")
# plt.ylabel("Count")
# plt.title("Distribution of Information Gain Advantage")

# plt.tight_layout()
# plt.savefig("auc_improvement_histogram_18.png", dpi=300)

# # =========================
# # Time-to-50 histogram
# # =========================
# plt.figure(figsize=(8,5))

# plt.hist(time_to_50_nom - time_to_50_occ, bins=20)
# plt.axvline(0, linestyle='--')

# plt.xlabel("Steps Faster to 50% Visibility (Positive = Occlusion Faster)")
# plt.ylabel("Count")
# plt.title("Speed Advantage to Reach 50% Visibility")

# plt.tight_layout()
# plt.savefig("time_to_50_advantage_18.png", dpi=300)

# print("\nAnalysis complete.")

import os
import re
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

# =========================
# USER CONFIG
# =========================
nominal_dir = '/home/intent/code/mighty_ws/src/mighty_sc/benchmark_data/18_nominal_vis'
occlusion_dir = '/home/intent/code/mighty_ws/src/mighty_sc/benchmark_data/18_vis'

output_prefix = "visibility_analysis"  # change this for saved files

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
# Storage
# =========================
all_improvements = []
auc_diffs = []

time_to_50_nom = []
time_to_50_occ = []

fractions_nominal = {0.25: [], 0.5: [], 0.75: [], 1.0: []}
fractions_occlusion = {0.25: [], 0.5: [], 0.75: [], 1.0: []}

max_length = 0

# =========================
# Helper functions
# =========================
def time_to_visibility(arr, threshold):
    for i, val in enumerate(arr):
        if val >= threshold:
            return i
    return len(arr)

def get_index(length, fraction):
    return min(int(fraction * length), length - 1)

# =========================
# First pass: max length
# =========================
for traj_id in nominal_files:
    if traj_id in occlusion_files:
        df_nom = pd.read_csv(nominal_files[traj_id], skiprows=3)
        df_occ = pd.read_csv(occlusion_files[traj_id], skiprows=3)

        max_length = max(max_length, len(df_nom), len(df_occ))

# =========================
# Main processing loop
# =========================
for traj_id in nominal_files:

    if traj_id not in occlusion_files:
        continue

    df_nom = pd.read_csv(nominal_files[traj_id], skiprows=3)
    df_occ = pd.read_csv(occlusion_files[traj_id], skiprows=3)

    perc_nom = np.array(df_nom.iloc[:, 1])
    perc_occ = np.array(df_occ.iloc[:, 1])

    if len(perc_nom) == 0 or len(perc_occ) == 0:
        continue

    # =========================
    # AUC
    # =========================
    auc_nom = np.trapz(perc_nom) / len(perc_nom)
    auc_occ = np.trapz(perc_occ) / len(perc_occ)
    auc_diffs.append(auc_occ - auc_nom)

    # =========================
    # Time to 50%
    # =========================
    time_to_50_nom.append(time_to_visibility(perc_nom, 50))
    time_to_50_occ.append(time_to_visibility(perc_occ, 50))

    # =========================
    # Improvement curve
    # =========================
    padded_nom = np.pad(perc_nom, (0, max_length - len(perc_nom)), mode='edge')
    padded_occ = np.pad(perc_occ, (0, max_length - len(perc_occ)), mode='edge')

    all_improvements.append(padded_occ - padded_nom)

    # =========================
    # NEW METRIC: self-normalized fractions
    # =========================
    T_nom = len(perc_nom)
    T_occ = len(perc_occ)

    final_nom = perc_nom[-1]
    final_occ = perc_occ[-1]

    if final_nom == 0 or final_occ == 0:
        continue

    for frac in [0.25, 0.5, 0.75, 1.0]:

        idx_nom = get_index(T_nom, frac)
        idx_occ = get_index(T_occ, frac)

        val_nom = perc_nom[idx_nom] / final_nom
        val_occ = perc_occ[idx_occ] / final_occ

        fractions_nominal[frac].append(val_nom)
        fractions_occlusion[frac].append(val_occ)

# =========================
# Convert to numpy
# =========================
all_improvements = np.array(all_improvements)
auc_diffs = np.array(auc_diffs)

time_to_50_nom = np.array(time_to_50_nom)
time_to_50_occ = np.array(time_to_50_occ)

# =========================
# PRINT STATISTICS
# =========================
print("\n===== AUC IMPROVEMENT =====")
print("Mean:", np.mean(auc_diffs))
print("Std:", np.std(auc_diffs))
print("Max:", np.max(auc_diffs))
print("Min:", np.min(auc_diffs))

print("\n===== TIME TO 50% VISIBILITY =====")
print("Nominal mean:", np.mean(time_to_50_nom))
print("Occlusion mean:", np.mean(time_to_50_occ))
print("Mean improvement:", np.mean(time_to_50_nom - time_to_50_occ))

print("\n===== SELF-NORMALIZED VISIBILITY =====")
for frac in [0.25, 0.5, 0.75, 1.0]:
    nom = np.array(fractions_nominal[frac])
    occ = np.array(fractions_occlusion[frac])

    print(f"\n--- {int(frac*100)}% of trajectory ---")
    print("Nominal:", np.mean(nom))
    print("Occlusion:", np.mean(occ))
    print("Improvement:", np.mean(occ - nom))

# =========================
# PLOT 1: Improvement curve
# =========================
mean_improvement = np.mean(all_improvements, axis=0)
std_improvement = np.std(all_improvements, axis=0)

x = np.arange(max_length)

plt.figure(figsize=(10,6))
plt.plot(x, mean_improvement, linewidth=3)
plt.fill_between(x, mean_improvement - std_improvement,
                 mean_improvement + std_improvement, alpha=0.3)
plt.axhline(0, linestyle='--')

plt.xlabel("Trajectory Index")
plt.ylabel("Visibility Improvement")
plt.title("Average Visibility Improvement")

plt.gca().xaxis.set_major_locator(ticker.MaxNLocator(6))
plt.gca().yaxis.set_major_locator(ticker.MaxNLocator(6))

plt.tight_layout()
plt.savefig(f"{output_prefix}_improvement_curve.png", dpi=300)

# =========================
# PLOT 2: AUC histogram
# =========================
plt.figure(figsize=(8,5))
plt.hist(auc_diffs, bins=20)
plt.axvline(0, linestyle='--')

plt.xlabel("AUC Improvement")
plt.ylabel("Count")
plt.title("AUC Difference Distribution")

plt.tight_layout()
plt.savefig(f"{output_prefix}_auc_hist.png", dpi=300)

# =========================
# PLOT 3: Time advantage
# =========================
plt.figure(figsize=(8,5))
plt.hist(time_to_50_nom - time_to_50_occ, bins=20)
plt.axvline(0, linestyle='--')

plt.xlabel("Steps Faster to 50%")
plt.ylabel("Count")
plt.title("Speed Advantage")

plt.tight_layout()
plt.savefig(f"{output_prefix}_time_advantage.png", dpi=300)

# =========================
# PLOT 4: Fractional progress
# =========================
fractions = [0.25, 0.5, 0.75, 1.0]

nom_means = [np.mean(fractions_nominal[f]) for f in fractions]
occ_means = [np.mean(fractions_occlusion[f]) for f in fractions]

plt.figure(figsize=(8,5))
plt.plot(fractions, nom_means, marker='o', label="Nominal")
plt.plot(fractions, occ_means, marker='o', label="Occlusion")

plt.xlabel("Fraction of Trajectory")
plt.ylabel("Fraction of Final Visibility")
plt.title("Visibility Progress Efficiency")

plt.legend()
plt.grid(True)

plt.tight_layout()
plt.savefig(f"{output_prefix}_fractional_progress.png", dpi=300)

print("\nAnalysis complete.")