import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

# Set seaborn style
sns.set_theme(style="whitegrid")

# Load logs
madgwick = pd.read_csv("./data/madgwick_log.txt", skiprows=1, sep=" ", names=[
    "timestamp", "roll", "pitch", "yaw", "pos_x", "pos_y", "pos_z", "vel_x", "vel_y", "vel_z"])
kalman = pd.read_csv("./data/kalman_log.txt", skiprows=1, sep=" ", names=[
    "timestamp", "roll", "pitch", "yaw", "pos_x", "pos_y", "pos_z", "vel_x", "vel_y", "vel_z"])
raw = pd.read_csv("./data/log.txt", skiprows=1, sep=" ", names=[
    "timestamp", "roll", "pitch", "yaw", "pos_x", "pos_y", "pos_z", "vel_x", "vel_y", "vel_z"])

# Normalize time
t0 = min(madgwick.timestamp.iloc[0], kalman.timestamp.iloc[0], raw.timestamp.iloc[0])
madgwick['time'] = madgwick['timestamp'] - t0
kalman['time'] = kalman['timestamp'] - t0
raw['time'] = raw['timestamp'] - t0

# X-axis range
x_min = 0
x_max = max(madgwick['time'].max(), kalman['time'].max(), raw['time'].max())

# X-axis ticks (e.g., every 5 seconds or finer based on data length)
xticks = range(int(x_min), int(x_max) + 1, 5)

# --- Plot Roll ---
plt.figure(figsize=(12, 5))
plt.plot(madgwick['time'], madgwick['roll'], label="Madgwick", linewidth=1.2)
plt.plot(kalman['time'], kalman['roll'], label="Kalman", linewidth=1.2)
plt.plot(raw['time'], raw['roll'], label="Raw", linewidth=1.2)
plt.title("Roll Over Time", fontsize=14)
plt.xlabel("Time (s)", fontsize=12)
plt.ylabel("Roll (°)", fontsize=12)
plt.xlim([x_min, x_max])
plt.xticks(xticks, rotation=45)
plt.legend()
plt.grid(True, linestyle='--', alpha=0.6)
plt.tight_layout()
plt.savefig("./data/roll_plot.png")

# --- Plot Position X ---
plt.figure(figsize=(12, 5))
plt.plot(madgwick['time'], madgwick['pos_x'], label="Madgwick", linewidth=1.2)
plt.plot(kalman['time'], kalman['pos_x'], label="Kalman", linewidth=1.2)
plt.plot(raw['time'], raw['pos_x'], label="Raw", linewidth=1.2)
plt.title("Position X Over Time", fontsize=14)
plt.xlabel("Time (s)", fontsize=12)
plt.ylabel("Position X (m)", fontsize=12)
plt.xlim([x_min, x_max])
plt.xticks(xticks, rotation=45)
plt.legend()
plt.grid(True, linestyle='--', alpha=0.6)
plt.tight_layout()
plt.savefig("./data/position_x_plot.png")

# --- Plot Position Y ---
plt.figure(figsize=(12, 5))
plt.plot(madgwick['time'], madgwick['pos_y'], label="Madgwick", linewidth=1.2)
plt.plot(kalman['time'], kalman['pos_y'], label="Kalman", linewidth=1.2)
plt.plot(raw['time'], raw['pos_y'], label="Raw", linewidth=1.2)
plt.title("Position Y Over Time", fontsize=14)
plt.xlabel("Time (s)", fontsize=12)
plt.ylabel("Position Y (m)", fontsize=12)
plt.xlim([x_min, x_max])
plt.xticks(xticks, rotation=45)
plt.legend()
plt.grid(True, linestyle='--', alpha=0.6)
plt.tight_layout()
plt.savefig("./data/position_y_plot.png")

# --- Plot Velocity Y ---
plt.figure(figsize=(12, 5))
plt.plot(madgwick['time'], madgwick['vel_y'], label="Madgwick", linewidth=1.2)
plt.plot(kalman['time'], kalman['vel_y'], label="Kalman", linewidth=1.2)
plt.plot(raw['time'], raw['vel_y'], label="Raw", linewidth=1.2)
plt.title("Velocity Y Over Time", fontsize=14)
plt.xlabel("Time (s)", fontsize=12)
plt.ylabel("Velocity Y (m/s)", fontsize=12)
plt.xlim([x_min, x_max])
plt.xticks(xticks, rotation=45)
plt.legend()
plt.grid(True, linestyle='--', alpha=0.6)
plt.tight_layout()
plt.savefig("./data/velocity_y_plot.png")

# --- Plot Pitch ---
plt.figure(figsize=(12, 5))
plt.plot(madgwick['time'], madgwick['pitch'], label="Madgwick", linewidth=1.2)
plt.plot(kalman['time'], kalman['pitch'], label="Kalman", linewidth=1.2)
plt.plot(raw['time'], raw['pitch'], label="Raw", linewidth=1.2)
plt.title("Pitch Over Time", fontsize=14)
plt.xlabel("Time (s)", fontsize=12)
plt.ylabel("Pitch (°)", fontsize=12)
plt.xlim([x_min, x_max])
plt.xticks(xticks, rotation=45)
plt.legend()
plt.grid(True, linestyle='--', alpha=0.6)
plt.tight_layout()
plt.savefig("./data/pitch_plot.png")

# --- Plot Velocity X ---
plt.figure(figsize=(12, 5))
plt.plot(madgwick['time'], madgwick['vel_x'], label="Madgwick", linewidth=1.2)
plt.plot(kalman['time'], kalman['vel_x'], label="Kalman", linewidth=1.2)
plt.plot(raw['time'], raw['vel_x'], label="Raw", linewidth=1.2)
plt.title("Velocity X Over Time", fontsize=14)
plt.xlabel("Time (s)", fontsize=12)
plt.ylabel("Velocity X (m/s)", fontsize=12)
plt.xlim([x_min, x_max])
plt.xticks(xticks, rotation=45)
plt.legend()
plt.grid(True, linestyle='--', alpha=0.6)
plt.tight_layout()
plt.savefig("./data/velocity_x_plot.png")

# --- Plot Velocity Z ---
plt.figure(figsize=(12, 5))
plt.plot(madgwick['time'], madgwick['vel_z'], label="Madgwick", linewidth=1.2)
plt.plot(kalman['time'], kalman['vel_z'], label="Kalman", linewidth=1.2)
plt.plot(raw['time'], raw['vel_z'], label="Raw", linewidth=1.2)
plt.title("Velocity Z Over Time", fontsize=14)
plt.xlabel("Time (s)", fontsize=12)
plt.ylabel("Velocity Z (m/s)", fontsize=12)
plt.xlim([x_min, x_max])
plt.xticks(xticks, rotation=45)
plt.legend()
plt.grid(True, linestyle='--', alpha=0.6)
plt.tight_layout()
plt.savefig("./data/velocity_z_plot.png")
