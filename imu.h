#ifndef IMU_H
#define IMU_H

#include <fstream>
#include <iostream>

#include "madgwick.h"
#include "data_structures.h"

class IMU {
  private:
    std::string log_filename = "../data/madgwick_log.txt";
    std::ofstream logfile;

    Relative_Position velocity {0, 0, 0};
    Relative_Position position {0, 0, 0};
    Madgwick madgwick;
    Orientation orientation;
    double current_time = 0;

  public:
    IMU();
    IMU(const std::string& log_filename);
    ~IMU();
    Relative_Position get_position();
    void receive_imu_data(double ax, double ay, double az, double gx, double gy, double gz, double timestamp);
};



#endif //IMU_H
