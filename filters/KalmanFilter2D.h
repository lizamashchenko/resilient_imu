#ifndef KALMAN_FILTER_2D_H
#define KALMAN_FILTER_2D_H

#include "IMUFilter.h"
#include "../data_structures.h"
#include "../linear_algebra/LinearAlgebra.h"

using namespace linear_algebra;

class KalmanFilter2D : public IMUFilter{
private:
    double angle_x, angle_y;
    double bias_x = 0.0;
    double bias_y = 0.0;
    double R;

    Matrix<double> P;  // 2x2
    Matrix<double> Q;  // 2x2
    Vector<double> H;  // 1x2

public:
    KalmanFilter2D(double initial_accel_x = 0, double initial_accel_y = 0, double initial_accel_z = 0, double R_ = 0.001);

    void fuse_data(Telemetry& telem, double dt) override;
    void update_orientation(Orientation &orient) override;
    void update_world_orientation(Telemetry &telem, Orientation orient, WorldOrientation &w_orient) override;

    double get_angle_x() const;
    double get_angle_y() const;
};

#endif // KALMAN_FILTER_2D_H
