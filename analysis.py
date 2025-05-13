import pandas as pd
import matplotlib.pyplot as plt

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

# --- Plot Roll ---
plt.figure(figsize=(10, 5))
plt.plot(madgwick['time'], madgwick['roll'], label="Madgwick")
plt.plot(kalman['time'], kalman['roll'], label="Kalman")
plt.plot(raw['time'], raw['roll'], label="Raw")
plt.title("Roll Over Time")
plt.xlabel("Time (s)")
plt.ylabel("Roll (degrees)")
plt.legend()
plt.grid()
plt.savefig("./data/roll_plot.png")

# --- Plot Position X ---
plt.figure(figsize=(10, 5))
plt.plot(madgwick['time'], madgwick['pos_x'], label="Madgwick")
plt.plot(kalman['time'], kalman['pos_x'], label="Kalman")
plt.plot(raw['time'], raw['pos_x'], label="Raw")
plt.title("Position X Over Time")
plt.xlabel("Time (s)")
plt.ylabel("Position X (m)")
plt.legend()
plt.grid()
plt.savefig("./data/position_x_plot.png")
