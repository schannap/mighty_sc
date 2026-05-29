# import os
# import re
# import pandas as pd
# import numpy as np
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D

# # === USER INPUT ===
# DATA_DIR = "/home/intent/code/mighty_ws/src/mighty_sc/benchmark_data/multi_thread"

# # Storage
# jerk_weights = []
# occ_weights = []
# mean_jerk_smoothness = []
# std_jerk_smoothness = []
# mean_cost = []
# std_cost = []

# pattern = re.compile(r"sweep_\d+\.csv$")

# for filename in os.listdir(DATA_DIR):
#     if pattern.search(filename):
#         filepath = os.path.join(DATA_DIR, filename)
#         with open(filepath, 'r') as f:
#             first_line = f.readline().strip()

#         # Extract weights from first line
#         # Example:
#         # jerk weight: 0.001, occlusion weight: -0.1
#         match = re.search(r"jerk weight:\s*([-\d.eE]+),\s*occlusion weight:\s*([-\d.eE]+)", first_line)
#         if not match:
#             print(f"Could not parse weights in {filename}")
#             continue

#         jerk_w = float(match.group(1))
#         occ_w = float(match.group(2))

#         # Read CSV skipping first line
#         df = pd.read_csv(filepath, skiprows=1)

#         # # Only successful trajectories
#         # df = df[df["success"] == 1]

#         # if len(df) == 0:
#         #     continue

#         jerk_mean = df["jerk_smoothness_l1"].mean()
#         jerk_std = df["jerk_smoothness_l1"].std()
#         cost_mean = df["cost_value"].mean()
#         cost_std = df["cost_value"].std()

#         jerk_weights.append(jerk_w)
#         occ_weights.append(occ_w)
#         mean_jerk_smoothness.append(jerk_mean)
#         std_jerk_smoothness.append(jerk_std)
#         mean_cost.append(cost_mean)
#         std_cost.append(cost_std)

# # Convert to numpy
# jerk_weights = np.array(jerk_weights)
# occ_weights = np.array(occ_weights)
# mean_jerk_smoothness = np.array(mean_jerk_smoothness)
# mean_cost = np.array(mean_cost)
# print(len(mean_jerk_smoothness))
# # === 3D Plot ===
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')

# sc = ax.scatter(jerk_weights,
#                 occ_weights,
#                 mean_jerk_smoothness,
#                 c=mean_jerk_smoothness,
#                 cmap='viridis')

# ax.set_xlabel("Jerk Weight")
# ax.set_ylabel("Occlusion Weight")
# ax.set_zlabel("Mean Jerk Smoothness (L1)")

# plt.colorbar(sc, label="Mean Jerk Smoothness")
# plt.title("Sweep Results: Jerk Smoothness vs Weights")
# plt.savefig("sweeps_smoothness_big_obs.png", dpi=300)


# # === 3D Plot ===
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')

# sc = ax.scatter(jerk_weights,
#                 occ_weights,
#                 mean_cost,
#                 c=mean_cost,
#                 cmap='viridis')

# ax.set_xlabel("Jerk Weight")
# ax.set_ylabel("Occlusion Weight")
# ax.set_zlabel("Mean Cost")

# plt.colorbar(sc, label="Mean Jerk Smoothness")
# plt.title("Sweep Results: Cost vs Weights")
# plt.savefig("sweeps_cost_big_obs.png", dpi=300)


import os
import re
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# === USER INPUT ===
DATA_DIR = "/home/intent/code/mighty_ws/src/mighty_sc/benchmark_data/multi_thread"

# =========================
# Signed log10 transform
# =========================
def signed_log10(x):
    x = np.array(x)
    return np.sign(x) * np.log10(np.abs(x))

# Storage
jerk_weights = []
occ_weights = []
mean_jerk_smoothness = []
std_jerk_smoothness = []
mean_cost = []
std_cost = []

pattern = re.compile(r"sweep_test_\d+\.csv$")

for filename in os.listdir(DATA_DIR):
    if pattern.search(filename):
        filepath = os.path.join(DATA_DIR, filename)

        with open(filepath, 'r') as f:
            first_line = f.readline().strip()

        # Extract weights
        match = re.search(
            r"jerk weight:\s*([-\d.eE]+),\s*occlusion weight:\s*([-\d.eE]+)",
            first_line
        )
        if not match:
            print(f"Could not parse weights in {filename}")
            continue

        jerk_w = float(match.group(1))
        occ_w = float(match.group(2))

        # Read CSV skipping first line
        df = pd.read_csv(filepath, skiprows=1)

        if len(df) == 0:
            continue

        jerk_mean = df["jerk_smoothness_l1"].mean()
        jerk_std = df["jerk_smoothness_l1"].std()
        cost_mean = df["cost_value"].mean()
        cost_std = df["cost_value"].std()

        jerk_weights.append(jerk_w)
        occ_weights.append(occ_w)
        mean_jerk_smoothness.append(jerk_mean)
        std_jerk_smoothness.append(jerk_std)
        mean_cost.append(cost_mean)
        std_cost.append(cost_std)

# Convert to numpy
jerk_weights = np.array(jerk_weights)
occ_weights = np.array(occ_weights)
mean_jerk_smoothness = np.array(mean_jerk_smoothness)
mean_cost = np.array(mean_cost)

print("Total sweep points:", len(mean_jerk_smoothness))

# =========================
# Apply signed log transform
# =========================
jerk_plot = signed_log10(jerk_weights)
occ_plot = np.log10(np.abs(occ_weights)) #signed_log10(occ_weights)

# =========================
# 3D Plot: Smoothness
# =========================
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

sc = ax.scatter(jerk_plot,
                occ_plot,
                mean_jerk_smoothness,
                c=mean_jerk_smoothness,
                cmap='viridis')

ax.set_xlabel("log10(Jerk Weight)")
ax.set_ylabel("Signed log10(Occlusion Weight)")
ax.set_zlabel("Mean Jerk Smoothness (L1)")

plt.colorbar(sc, label="Mean Jerk Smoothness")
plt.title("Sweep Results: Jerk Smoothness vs Weights")
plt.savefig("sweeps_smoothness_signed_log_bigobs.png", dpi=300)
plt.close()

# =========================
# 3D Plot: Cost
# =========================
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

sc = ax.scatter(jerk_plot,
                occ_plot,
                mean_cost,
                c=mean_cost,
                cmap='viridis')

ax.set_xlabel("log10(Jerk Weight)")
ax.set_ylabel("Signed log10(Occlusion Weight)")
ax.set_zlabel("Mean Cost")

plt.colorbar(sc, label="Mean Cost")
plt.title("Sweep Results: Cost vs Weights")
plt.savefig("sweeps_cost_signed_log_bigobs.png", dpi=300)
plt.close()

print(mean_cost)

summary_cost_log = pd.DataFrame({
    "log10_jerk_weight": np.log10(jerk_weights),
    "log10_abs_occlusion_weight": np.log10(np.abs(occ_weights)),
    "mean_cost": mean_cost
}).sort_values(by=["log10_jerk_weight",
                   "log10_abs_occlusion_weight"])

print("\n===== COST SUMMARY (log space) =====")
print(summary_cost_log.to_string(index=False))

summary_smooth_log = pd.DataFrame({
    "log10_jerk_weight": np.log10(jerk_weights),
    "log10_abs_occlusion_weight": np.log10(np.abs(occ_weights)),
    "mean_jerk_smoothness": mean_jerk_smoothness
}).sort_values(by=["log10_jerk_weight",
                   "log10_abs_occlusion_weight"])

print("\n===== SMOOTHNESS SUMMARY (log space) =====")
print(summary_smooth_log.to_string(index=False))

print("Min abs occ weight:", np.min(np.abs(occ_weights)))
print("Max abs occ weight:", np.max(np.abs(occ_weights)))