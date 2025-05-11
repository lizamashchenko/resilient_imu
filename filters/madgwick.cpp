//
// Created by liza on 09.04.25.
//

#include "madgwick.h"

void Madgwick::fuse_data(Telemetry& telem, double dt) {
    Vector<double> accel_v {telem.ax, telem.ay, telem.az};
    Vector<double> gyro_v {telem.gx, telem.gy, telem.gz};

    accel_v.normalize();

    Vector<double> q_dot {
    	(-q[1] * telem.gx - q[2] * telem.gy - q[3] * telem.gz) / 2.0,
    	(q[0] * telem.gx - q[3] * telem.gy + q[2] * telem.gz) / 2.0,
    	(q[3] * telem.gx + q[0] * telem.gy - q[1] * telem.gz) / 2.0,
    	(-q[2] * telem.gx + q[1] * telem.gy + q[0] * telem.gz) / 2.0
    };

    Vector<double> f {
		2 * (q[1] * q[3] - q[0] * q[2]) - accel_v[0],
		2 * (q[0] * q[1] - q[2] * q[3]) - accel_v[1],
		2 * (0.5 - q[1] * q[1] - q[2] * q[2]) - accel_v[2]
    };

    Matrix<double> j;
    j.add_row({-2 * q[2], 2 * q[3], -2 * q[0], 2 * q[1]});
    j.add_row({2 * q[1], 2 * q[0], 2 * q[3], 2 * q[2]});
    j.add_row({0, -4 * q[1], -4 * q[2], 0});

    Vector<double> step = j.transpose() * f;
    step.normalize();

    q_dot = q_dot - (step * beta);
    q = q + (q_dot * dt);
    q.normalize();
}

const Vector<double>& Madgwick::get_quaternion() const { return q; }
