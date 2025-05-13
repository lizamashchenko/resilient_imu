#ifndef MADGWICK_FILTER_H
#define MADGWICK_FILTER_H

#include "IMUFilter.h"
#include "../data_structures.h"
#include "../config/configuration.h"
#include "../linear_algebra/LinearAlgebra.h"

using namespace linear_algebra;

class MadgwickFilter : public IMUFilter {
public:
  void fuse_data(Telemetry& telem, double dt) override;
  void update_orientation(Orientation& orient) override;
  void update_world_orientation(Telemetry& telem, Orientation orient, WorldOrientation& w_orient) override;
  [[nodiscard]] const Vector<double>& get_quaternion() const;
private:
  Vector<double> q = {1.0, 0.0, 0.0, 0.0};
  double beta = MADGWICK_BETA;
  double alpha = MADGWICK_ALPHA; // Smoothing factor for low-pass filter
  Vector<double> accel_filtered = {0.0, 0.0, 0.0};

  [[nodiscard]] Vector<double> low_pass_filter(const Vector<double>& current, const Vector<double>& previous) const {
    return previous * alpha + current * (1.0 - alpha);
  }

  static double adaptive_beta(const Vector<double>& accel) {
    double accel_magnitude = std::sqrt(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
    return 0.1 + (accel_magnitude - 9.81) * 0.05;
  }
};




#endif //MADGWICK_FILTER_H
