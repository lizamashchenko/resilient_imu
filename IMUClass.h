#ifndef IMU_H
#define IMU_H

#include <fstream>
#include <iostream>
#include <memory>

#include "filters/MadgwickFilter.h"
#include "filters/KalmanFilter2D.h"
#include "data_structures.h"

#ifdef USE_MADGWICK
#include "./filters/MadgwickFilter.h"
typedef MadgwickFilter SelectedFilter;
#else
#include "./filters/KalmanFilter2D.h"
typedef KalmanFilter2D SelectedFilter;
#endif


class IMU {
private:
    std::string log_filename = "../data/log_file.txt";
    std::ofstream logfile;

    Vector3D velocity {0, 0, 0};
    Vector3D position {0, 0, 0};
    Orientation orientation;
    WorldOrientation world_orientation;
    SelectedFilter filter;
    double current_time = 0;

    void log_state(double timestamp);

public:
    IMU();
    explicit IMU(const std::string& log_filename);
    ~IMU();

    void receive_imu_data(double ax, double ay, double az,
                      double gx, double gy, double gz,
                      double timestamp);
};



#endif //IMU_H
