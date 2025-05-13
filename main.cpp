#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include "IMUClass.h"

constexpr double time_interval = 0.1;

int main() {
    std::string input_file = "../data/log.txt";
    std::ifstream infile(input_file);

    if (!infile.is_open()) {
        std::cerr << "Failed to open input file: " << input_file << std::endl;
        return 1;
    }

#ifdef USE_MADGWICK
    IMU imu_logger("../data/madgwick_log.txt");
#elif USE_KALMAN
    IMU imu_logger("../data/kalman_log.txt");
#elif USE_RAW
    IMU imu_logger("../data/raw_log.txt");
#endif

    std::string line;
    std::getline(infile, line);
    int line_index = 0;


    while (std::getline(infile, line)) {
        std::istringstream iss(line);
        double ax, ay, az;
        double gx, gy, gz;
        double mx, my, mz;
        double heading, roll, pitch;
        double qw, qx, qy, qz;

        if (!(iss >> ax >> ay >> az >> gx >> gy >> gz >>
              mx >> my >> mz >>
              heading >> roll >> pitch >>
              qw >> qx >> qy >> qz)) {
            std::cerr << "Invalid line: " << line << std::endl;
            continue;
        }
        double timestamp = line_index * time_interval;
        line_index++;
        imu_logger.receive_imu_data(ax, ay, az, gx, gy, gz, timestamp);
    }

    return 0;
}