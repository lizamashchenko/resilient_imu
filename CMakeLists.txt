    cmake_minimum_required(VERSION 3.30)
    SET(PROJECT_NAME resilient_imu)

    set(CMAKE_CXX_STANDARD 20)

    set(IMU_FILTER "MADGWICK" CACHE STRING "Choose IMU filter: MADGWICK or KALMAN")

    if (IMU_FILTER STREQUAL "MADGWICK")
        add_compile_definitions(USE_MADGWICK)
        message("USING MADGWICK")
    elseif(IMU_FILTER STREQUAL "KALMAN")
        add_compile_definitions(USE_KALMAN)
        message("USING KALMAN")
    else()
        message(FATAL_ERROR "Invalid IMU_FILTER value. Choose MADGWICK or KALMAN.")
    endif()


    add_executable(${PROJECT_NAME} main.cpp
            filters/MadgwickFilter.h
            filters/MadgwickFilter.cpp
            IMUClass.h
            IMUClass.cpp
            filters/KalmanFilter2D.cpp
            filters/KalmanFilter2D.h
            linear_algebra/LinearAlgebra.h
            filters/IMUFilter.h
    )
