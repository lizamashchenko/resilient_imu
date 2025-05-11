#ifndef IMU_H
#define IMU_H

#include <fstream>
#include <iostream>

#include "filters/madgwick.h"
#include "filters/kalman.h"
#include "data_structures.h"

class IMU {
  private:
    std::string log_filename = "../data/log_file.txt";
    std::ofstream logfile;

    Relative_Position velocity {0, 0, 0};
    Relative_Position position {0, 0, 0};
    Madgwick madgwick;
    KalmanFilter2D kalman;
    Orientation orientation;
    double current_time = 0;

  public:
    IMU();
    IMU(const std::string& log_filename);
    ~IMU();
    Relative_Position get_position();
    void receive_imu_data_madgwick(double ax, double ay, double az, double gx, double gy, double gz, double timestamp);
    void receive_imu_data_kalman(double ax, double ay, double az, double gx, double gy, double gz, double timestamp);
};



#endif //IMU_H
