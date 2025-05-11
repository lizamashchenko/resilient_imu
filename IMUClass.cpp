#include "IMUClass.h"

#include <utility>

IMU::IMU() {
    logfile.open(log_filename);
    if (!logfile.is_open()) {
        throw std::runtime_error("Failed to open IMU log file");
    }

    logfile << "timestamp,roll,pitch,yaw,pos_x,pos_y,pos_z,vel_x,vel_y,vel_z\n";
}
IMU::IMU(const std::string& log_filename) {
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

void IMU::receive_imu_data(const double ax, const double ay, const double az,
                           const double gx, const double gy, const double gz,
                           const double timestamp) {
    Telemetry new_telem;
    const double dt = timestamp - current_time;
    new_telem.ax = ax;
    new_telem.ay = ay;
    new_telem.az = az;
    new_telem.gx = gx;
    new_telem.gy = gy;
    new_telem.gz = gz;

    filter.fuse_data(new_telem, dt);
    filter.update_orientation(orientation);
    filter.update_world_orientation(new_telem, orientation, world_orientation);

    velocity.x += world_orientation.a_world_x * dt;
    velocity.y += world_orientation.a_world_y * dt;
    velocity.z += world_orientation.a_world_z * dt;

    position.x += velocity.x * dt;
    position.y += velocity.y * dt;
    position.z += velocity.z * dt;

    current_time = timestamp;
    log_state(timestamp);
}

void IMU::log_state(const double timestamp) {
    logfile << timestamp << " "
        << orientation.roll << " "
        << orientation.pitch << " "
        << orientation.yaw << " "
        << position.x << " " << position.y << " " << position.z << " "
        << velocity.x << " " << velocity.y << " " << velocity.z << "\n";
}


