import numpy as np
import pandas as pd

file_path = "data/imu_data.txt"
df = pd.read_csv(file_path, sep=" ", header=None)
df.columns = ["timestamp", "gx", "gy", "gz", "ax", "ay", "az", "temp"]
df["timestamp"] = (df["timestamp"] - df["timestamp"].min()) / 1e6

beta = 0.1
dt = df["dt"].mean()

q = np.array([1.0, 0.0, 0.0, 0.0])

madgwick_angle_x = []
madgwick_angle_y = []

def normalize(v):
    norm = np.linalg.norm(v)
    return v / norm if norm != 0 else v

def madgwick_update(gyro, accel, q, beta, dt):
    gx, gy, gz = gyro
    ax, ay, az = accel

    accel = normalize(np.array([ax, ay, az]))

    q_dot = 0.5 * np.array([
        -q[1] * gx - q[2] * gy - q[3] * gz,
        q[0] * gx + q[2] * gz - q[3] * gy,
        q[0] * gy - q[1] * gz + q[3] * gx,
        q[0] * gz + q[1] * gy - q[2] * gx
    ])

    f = np.array([
        2 * (q[1] * q[3] - q[0] * q[2]) - accel[0],
        2 * (q[0] * q[1] - q[2] * q[3]) - accel[1],
        2 * (0.5 - q[1] * 2 - q[2] * 2) - accel[2]
    ])

    j = np.array([
        [-2 * q[2], 2 * q[3], -2 * q[0], 2 * q[1]],
        [2 * q[1], 2 * q[0], 2 * q[3], 2 * q[2]],
        [0, -4 * q[1], -4 * q[2], 0]
    ])

    step = normalize(j.T @ f)

    q_dot -= beta * step

    q += q_dot * dt
    q = normalize(q)

    return q

for i in range(len(df)):
    gyro = [df["gx"].iloc[i], df["gy"].iloc[i], df["gz"].iloc[1]]
    accel = [df["ax"].iloc[i], df["ay"].iloc[i], df["az"].iloc[1]]

    q = madgwick_update(gyro, accel, q, beta, dt)

    roll = np.degrees(np.arctan2(2 * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * (q[1] * 2 + q[2] * 2)))
    pitch = np.degrees(np.arcsin(2 * (q[0] * q[2] - q[3] * q[1])))

    madgwick_angle_x.append(roll)
    madgwick_angle_y.append(pitch)

df["madgwick_angle_x"] = madgwick_angle_x
df["madgwick_angle_y"] = madgwick_angle_y

fig = px.line(df, x="timestamp", y=["madgwick_angle_x", "madgwick_angle_y"],
            title="Madgwick Filtered Angles")
fig.update_layout(xaxis_title="Time(s)", yaxis_title="Angle(grad)", xaxis_rangeslider_visible=True)
fig.show()
