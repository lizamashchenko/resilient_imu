    cmake_minimum_required(VERSION 3.30)
    SET(PROJECT_NAME resilient_imu)

    set(CMAKE_CXX_STANDARD 20)

    set(SOURCES main.cpp IMUClass.cpp)
    set(IMU_FILTER "MADGWICK" CACHE STRING "Choose IMU filter: MADGWICK or KALMAN or RAW")

    if (IMU_FILTER STREQUAL "MADGWICK")
        add_compile_definitions(USE_MADGWICK)
        message("USING MADGWICK")
        list(APPEND SOURCES filters/MadgwickFilter.cpp)
    elseif(IMU_FILTER STREQUAL "KALMAN")
        add_compile_definitions(USE_KALMAN)
        message("USING KALMAN")
        list(APPEND SOURCES filters/KalmanFilter2D.cpp)
    elseif (IMU_FILTER STREQUAL "RAW")
        add_compile_definitions(USE_RAW)
        message("USING RAW")
        list(APPEND SOURCES filters/RawFilter.cpp)
    else()
        message(FATAL_ERROR "Invalid IMU_FILTER value. Choose MADGWICK or KALMAN or RAW")
    endif()

    add_executable(${PROJECT_NAME} ${SOURCES})
    target_include_directories(${PROJECT_NAME} PRIVATE filters linear_algebra config)

