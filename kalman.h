//
// Created by liza on 09.05.25.
//

#ifndef KALMAN_H
#define KALMAN_H
#include "Matrix.h"


class kalman {
private:
    double bias_x = 0.0;
    double bias_y = 0.0;
    double R = 0.03;

    Matrix<double> P;
    Matrix<double> Q;
    Vector<double> h;

public:
    kalman()
    : P(2, 2, {{1.0, 0.0}, {0.0, 1.0}}),
      Q(2, 2, {{0.001, 0.0}, {0.0, 0.003}}),
      h({1.0, 0.0})
    {}


};



#endif //KALMAN_H
