# 🌐 Resilient IMU System

**Resilient IMU** is a simulation tool that mimics the behavior of firmware when fusing data from an Inertial Measurement Unit (IMU). The project supports multiple fusion filters and logs the resulting orientation, velocity, and position over time.

---

## 📄 Input Format

The input data file is hardcoded in `main.cpp` and must be formatted as space-separated lines:

```
ax ay az gx gy gz mx my mz heading roll pitch qw qx qy qz
ax ay az gx gy gz mx my mz heading roll pitch qw qx qy qz
...
```

> ℹ️ **Note:** The format is subject to change—check `main.cpp` for the most up-to-date expectations.

---

## 📦 IMU Class

The core of the system is the `IMU` class, which:

- Fuses incoming IMU telemetry using the selected filter
- Maintains internal state: orientation, velocity, and position
- Logs output to a `.txt` file for further analysis

---

## ⚙️ Configuration Header

The configuration header (in `config/`) sets:

- Initial filter parameters
- Starting conditions (orientation, position, etc.)
- Gravity compensation settings

---

## 🛠️ Compilation

To build the project with a specific filter, use the `IMU_FILTER` CMake variable.

### 🔧 Example Commands

```bash
# Build with Madgwick filter
cmake -B build -DIMU_FILTER=MADGWICK
cmake --build build

# Build with Kalman filter
cmake -B build -DIMU_FILTER=KALMAN
cmake --build build

# Build with raw (no fusion) filter
cmake -B build -DIMU_FILTER=RAW
cmake --build build
```

> 🧼 Tip: Use separate build directories to avoid conflicts when switching filters.

---

## 📁 Project Structure (Simplified)

```
resilient_imu/
├── CMakeLists.txt
├── main.cpp
├── IMUClass.cpp / .h
├── filters/
│   ├── MadgwickFilter.cpp / .h
│   ├── KalmanFilter2D.cpp / .h
│   └── RawFilter.cpp / .h
├── linear_algebra/
│   └── LinearAlgebra.h
├── config/
│   └── filter_config.h
├── data/
│   └── imu_data.txt
```

---

## 🧪 Output

Output is saved as a CSV-style text file containing:

```
timestamp, roll, pitch, yaw, pos_x, pos_y, pos_z, vel_x, vel_y, vel_z
```

The file name depends on the selected filter (e.g., `madgwick_log.txt`, `kalman_log.txt`).

---
