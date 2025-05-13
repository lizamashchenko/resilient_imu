#ifndef CONFIGURATION_H
#define CONFIGURATION_H

constexpr double MADGWICK_BETA = 5.0;
constexpr double MADGWICK_ALPHA = 0.8;

constexpr double KALMAN_R_MEASURE = 0.0003;
constexpr double KALMAN_Q_ANGLE = 0.001;
constexpr double KALMAN_Q_BIAS = 0.003;

constexpr double KALMAN_INITIAL_ANGLE_X = 0;
constexpr double KALMAN_INITIAL_ANGLE_Y = 0;
constexpr double KALMAN_INITIAL_ANGLE_Z = 0;

constexpr double GRAVITY = 9.81;

#endif //CONFIGURATION_H
