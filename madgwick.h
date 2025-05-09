//
// Created by liza on 09.04.25.
//

#ifndef MADGWICK_H
#define MADGWICK_H

#include "data_structures.h"
#include <stdlib.h>
#include "Vector.h"

class Madgwick {
private:
  Vector<double> q {1.0, 0, 0, 0};
  const double beta = 0.1;

public:
  void fuse_data(Telemetry& telem, double dt);
  const Vector<double>& get_quaternion() const;
};



#endif //MADGWICK_H
