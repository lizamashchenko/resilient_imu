#ifndef KALMAN_FILTER_2D_H
#define KALMAN_FILTER_2D_H

#include "../data_structures.h"
#include "../linear_algebra/linear_algebra.h"

using namespace linear_algebra;

class KalmanFilter2D {
private:
    double angle_x, angle_y;
    double bias_x = 0.0;
    double bias_y = 0.0;
    double R;

    Matrix<double> P;  // 2x2
    Matrix<double> Q;  // 2x2
    Vector<double> H;  // 1x2

public:
    KalmanFilter2D(double initial_accel_x, double initial_accel_y, double initial_accel_z, double R_ = 0.03)
        : R(R_),
          P(2, 2, {{1.0, 0.0}, {0.0, 1.0}}),
          Q(2, 2, {{0.001, 0.0}, {0.0, 0.003}}),
          H({1.0, 0.0})
    {
        angle_x = std::atan2(initial_accel_y, initial_accel_z) * 180.0 / M_PI;
        angle_y = std::atan2(-initial_accel_x, std::sqrt(initial_accel_y * initial_accel_y + initial_accel_z * initial_accel_z)) * 180.0 / M_PI;
    }

    std::pair<double, double> fuse_data(Telemetry& telem, double dt) {
        angle_x += (telem.gx - bias_x) * dt;
        angle_y += (telem.gy - bias_y) * dt;
        P = P + Q;

        double accel_angle_x = std::atan2(telem.ay, telem.az) * 180.0 / M_PI;
        double accel_angle_y = std::atan2(-telem.ax, std::sqrt(telem.ay * telem.ay + telem.az * telem.az)) * 180.0 / M_PI;

        // Kalman Gain: K = P * H^T / (H * P * H^T + R)
        Matrix<double> H_T = H.transpose_as_column();           // 2x1
        Matrix<double> PHt = P * H_T;                 // 2x1
        double denom = (H * PHt)[0] + R;              // scalar
        Matrix<double> K = PHt / denom;               // 2x1

        // Update step
        double innovation_x = accel_angle_x - angle_x;
        double innovation_y = accel_angle_y - angle_y;

        // fix to conversion to vector
        angle_x += K[0][0] * innovation_x;
        angle_y += K[0][0] * innovation_y;
        bias_x += K[1][0] * innovation_x;
        bias_y += K[1][0] * innovation_y;

        // P = (I - K * H) * P
        Matrix<double> I(2, 2);
        I.set(0, 0, 1.0); I.set(1, 1, 1.0);
        Matrix<double> KH(2, 2);
        for (int i = 0; i < 2; ++i)
            for (int j = 0; j < 2; ++j)
                KH[i][j] = K[i][0] * H[j];

        Matrix<double> I_KH = I - KH;
        P = I_KH * P;

        return {angle_x, angle_y};
    }
    double get_angle_x() const { return angle_x; }
    double get_angle_y() const { return angle_y; }
};

#endif // KALMAN_FILTER_2D_H
