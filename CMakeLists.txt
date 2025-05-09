cmake_minimum_required(VERSION 3.30)
SET(PROJECT_NAME resilient_imu)

set(CMAKE_CXX_STANDARD 20)

add_executable(${PROJECT_NAME} main.cpp
        madgwick.h
        madgwick.cpp
        Matrix.h
        Vector.h
        imu.h
        imu.cpp
        kalman.cpp
        kalman.h
)
