#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include "imu.h"

int main() {
    std::string input_file = "../data/imu_data.txt";
    std::ifstream infile(input_file);

    if (!infile.is_open()) {
        std::cerr << "Failed to open input file: " << input_file << std::endl;
        return 1;
    }

    IMU imu_logger_madgwick("../data/madgwick_log.txt");
    IMU imu_logger_kalman ("../data/kalman_log.txt");
    std::string line;
    while (std::getline(infile, line)) {
        std::istringstream iss(line);
        double timestamp, gx, gy, gz, ax, ay, az, temp;

        if (!(iss >> timestamp >> gx >> gy >> gz >> ax >> ay >> az >> temp)) {
            std::cerr << "Invalid line: " << line << std::endl;
            continue;
        }

        imu_logger_kalman.receive_imu_data_kalman(ax, ay, az, gx, gy, gz, timestamp);
        imu_logger_madgwick.receive_imu_data_madgwick(ax, ay, az, gx, gy, gz, timestamp);
    }

    return 0;
}
