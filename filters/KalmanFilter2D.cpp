#include "KalmanFilter2D.h"

KalmanFilter2D::KalmanFilter2D(double initial_accel_x, double initial_accel_y, double initial_accel_z, double R_)
    : R(R_),
      P(2, 2, {{1.0, 0.0}, {0.0, 1.0}}),
      Q(2, 2, {{0.1, 0.0}, {0.0, 0.3}}),
      H({1.0, 0.0})
{
    angle_x = std::atan2(initial_accel_y, initial_accel_z) * 180.0 / M_PI;
    angle_y = std::atan2(-initial_accel_x, std::sqrt(initial_accel_y * initial_accel_y + initial_accel_z * initial_accel_z)) * 180.0 / M_PI;
}

void KalmanFilter2D::fuse_data(Telemetry& telem, const double dt) {
    angle_x += (telem.gx - bias_x) * dt;
    angle_y += (telem.gy - bias_y) * dt;
    P = P + Q;

    double accel_angle_x = std::atan2(telem.ay, telem.az) * 180.0 / M_PI;
    double accel_angle_y = std::atan2(-telem.ax, std::sqrt(telem.ay * telem.ay + telem.az * telem.az)) * 180.0 / M_PI;

    // Kalman Gain: K = P * H^T / (H * P * H^T + R)
    Matrix<double> H_T = H.transpose_as_column();           // 2x1
    Matrix<double> PHt = P * H_T;                           // 2x1
    double denom = (H * PHt)[0] + R;                        // scalar
    Matrix<double> K = PHt / denom;                         // 2x1

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
}

void KalmanFilter2D::update_orientation(Orientation &orient) {
    orient.roll = angle_x * M_PI / 180.0;
    orient.pitch = angle_y * M_PI / 180.0;
}

void KalmanFilter2D::update_world_orientation(Telemetry &telem, Orientation orient, WorldOrientation &w_orient) {
    const double sin_r = sin(orient.roll);
    const double cos_r = cos(orient.roll);
    const double sin_p = sin(orient.pitch );
    const double cos_p = cos(orient.pitch );

    w_orient.a_world_x = telem.ax * cos_p + telem.az * sin_p;
    w_orient.a_world_y = telem.ax * sin_r * sin_p + telem.ay * cos_r - telem.az * sin_r * cos_p;
    w_orient.a_world_z = -telem.ax * cos_r * sin_p + telem.ay * sin_r + telem.az * cos_r * cos_p - 9.81;
}


double KalmanFilter2D::get_angle_x() const { return angle_x; }
double KalmanFilter2D::get_angle_y() const { return angle_y; }