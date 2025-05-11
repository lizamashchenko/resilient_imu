#ifndef MADGWICK_FILTER_H
#define MADGWICK_FILTER_H

#include "IMUFilter.h"
#include "../data_structures.h"
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
  double beta = 0.1;
};




#endif //MADGWICK_FILTER_H
