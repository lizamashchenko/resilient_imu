cmake_minimum_required(VERSION 3.30)
SET(PROJECT_NAME resilient_imu)

set(CMAKE_CXX_STANDARD 20)

add_executable(${PROJECT_NAME} main.cpp
        filters/madgwick.h
        filters/madgwick.cpp
        imu.h
        imu.cpp
        filters/kalman.cpp
        filters/kalman.h
        linear_algebra/linear_algebra.h
)
