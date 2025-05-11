#include "imu.h"

#include <utility>

IMU::IMU(): kalman(0,0, 0) {
    logfile.open(log_filename);
    if (!logfile.is_open()) {
        throw std::runtime_error("Failed to open IMU log file");
    }

    logfile << "timestamp,roll,pitch,yaw,pos_x,pos_y,pos_z,vel_x,vel_y,vel_z\n";
}
IMU::IMU(const std::string& log_filename): kalman(0, 0, 0) {
    logfile.open(log_filename);
    if (!logfile.is_open()) {
        throw std::runtime_error("Failed to open IMU log file");
    }

    logfile << "timestamp,roll,pitch,yaw,pos_x,pos_y,pos_z,vel_x,vel_y,vel_z\n";
}

IMU::~IMU() {
    if (logfile.is_open()) {
        logfile.close();
    }
}

void IMU::receive_imu_data_madgwick(double ax, double ay, double az, double gx, double gy, double gz, double timestamp) {
    Telemetry new_telem;
    double dt = timestamp - current_time;
    new_telem.ax = ax;
    new_telem.ay = ay;
    new_telem.az = az;
    new_telem.gx = gx;
    new_telem.gy = gy;
    new_telem.gz = gz;

    madgwick.fuse_data(new_telem, dt);
    Vector<double> quaternion = madgwick.get_quaternion();
    std::cout << quaternion[0] << " " << quaternion[1] << " " << quaternion[2] << " " << quaternion[3] << std::endl;

    orientation.roll = std::atan2(2.0 * (quaternion[0] * quaternion[1] + quaternion[2] * quaternion[3]),
                                  1.0 - 2.0 * (quaternion[1] * quaternion[1] + quaternion[2] * quaternion[2]));
    orientation.pitch = std::asin(2.0 * (quaternion[0] * quaternion[2] - quaternion[3] * quaternion[1]));
    orientation.yaw = std::atan2(2.0 * (quaternion[0] * quaternion[3] + quaternion[1] * quaternion[2]),
                                 1.0 - 2.0 * (quaternion[2] * quaternion[2] + quaternion[3] * quaternion[3]));

    double qw = quaternion[0], qx = quaternion[1], qy = quaternion[2], qz = quaternion[3];

    double a_world_x = ax * (1 - 2*qy*qy - 2*qz*qz) + ay * (2*qx*qy - 2*qz*qw) + az * (2*qx*qz + 2*qy*qw);
    double a_world_y = ax * (2*qx*qy + 2*qz*qw) + ay * (1 - 2*qx*qx - 2*qz*qz) + az * (2*qy*qz - 2*qx*qw);
    double a_world_z = ax * (2*qx*qz - 2*qy*qw) + ay * (2*qy*qz + 2*qx*qw) + az * (1 - 2*qx*qx - 2*qy*qy);

    a_world_z -= 9.81;

    velocity.x += a_world_x * dt;
    velocity.y += a_world_y * dt;
    velocity.z += a_world_z * dt;

    position.x += velocity.x * dt;
    position.y += velocity.y * dt;
    position.z += velocity.z * dt;

    logfile << timestamp << " "
        << orientation.roll << " "
        << orientation.pitch << " "
        << orientation.yaw << " "
        << position.x << " " << position.y << " " << position.z << " "
        << velocity.x << " " << velocity.y << " " << velocity.z << "\n";

    current_time = timestamp;
}

void IMU::receive_imu_data_kalman(double ax, double ay, double az, double gx, double gy, double gz, double timestamp) {
    Telemetry new_telem;
    double dt = timestamp - current_time;
    new_telem.ax = ax;
    new_telem.ay = ay;
    new_telem.az = az;
    new_telem.gx = gx;
    new_telem.gy = gy;
    new_telem.gz = gz;

    kalman.fuse_data(new_telem, dt);

    // Using the Kalman filter results for orientation calculation
    double angle_x = kalman.get_angle_x();
    double angle_y = kalman.get_angle_y();

    // The yaw angle can be derived based on your specific setup (it could be integrated, or derived from other sensor data if available).
    // For simplicity, assuming yaw is 0 in this example.
    orientation.roll = angle_x * M_PI / 180.0;
    orientation.pitch = angle_y * M_PI / 180.0;

    double sin_r = sin(orientation.roll);
    double cos_r = cos(orientation.roll);
    double sin_p = sin(orientation.pitch );
    double cos_p = cos(orientation.pitch );

    // Rotation from body to world (neglecting yaw)
    double a_world_x = ax * cos_p + az * sin_p;
    double a_world_y = ax * sin_r * sin_p + ay * cos_r - az * sin_r * cos_p;
    double a_world_z = -ax * cos_r * sin_p + ay * sin_r + az * cos_r * cos_p;

    a_world_z -= 9.81;

    velocity.x += a_world_x * dt;
    velocity.y += a_world_y * dt;
    velocity.z += a_world_z * dt;

    position.x += velocity.x * dt;
    position.y += velocity.y * dt;
    position.z += velocity.z * dt;

    logfile << timestamp << " "
        << orientation.roll << " "
        << orientation.pitch << " "
        << orientation.yaw << " "
        << position.x << " " << position.y << " " << position.z << " "
        << velocity.x << " " << velocity.y << " " << velocity.z << "\n";

    current_time = timestamp;
}


