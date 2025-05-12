#include "MadgwickFilter.h"

void MadgwickFilter::fuse_data(Telemetry& telem, double dt) {
    Vector accel_v {telem.ax, telem.ay, telem.az};
    Vector gyro_v {telem.gx, telem.gy, telem.gz};


	accel_filtered = low_pass_filter(accel_v, accel_filtered);
	accel_filtered.normalize();

	beta = adaptive_beta(accel_filtered);
    accel_v.normalize();

    Vector q_dot {
    	(-q[1] * telem.gx - q[2] * telem.gy - q[3] * telem.gz) / 2.0,
    	(q[0] * telem.gx - q[3] * telem.gy + q[2] * telem.gz) / 2.0,
    	(q[3] * telem.gx + q[0] * telem.gy - q[1] * telem.gz) / 2.0,
    	(-q[2] * telem.gx + q[1] * telem.gy + q[0] * telem.gz) / 2.0
    };

    Vector f {
		2 * (q[1] * q[3] - q[0] * q[2]) - accel_filtered[0],
		2 * (q[0] * q[1] - q[2] * q[3]) - accel_filtered[1],
		2 * (0.5 - q[1] * q[1] - q[2] * q[2]) - accel_filtered[2]
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

void MadgwickFilter::update_orientation(Orientation& orient) {
	orient.roll = std::atan2(2.0 * (q[0] * q[1] + q[2] * q[3]),
								 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]));
	orient.pitch = std::asin(2.0 * (q[0] * q[2] - q[3] * q[1]));
	orient.yaw = std::atan2(2.0 * (q[0] * q[3] + q[1] * q[2]),
								 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]));
}

void MadgwickFilter::update_world_orientation(Telemetry& telem, Orientation orient, WorldOrientation& w_orient) {
	const double qw = q[0];
	const double qx = q[1];
	const double qy = q[2];
	const double qz = q[3];

	w_orient.a_world_x = telem.ax * (1 - 2*qy*qy - 2*qz*qz) + telem.ay * (2*qx*qy - 2*qz*qw) + telem.az * (2*qx*qz + 2*qy*qw);
	w_orient.a_world_y = telem.ax * (2*qx*qy + 2*qz*qw) + telem.ay * (1 - 2*qx*qx - 2*qz*qz) + telem.az * (2*qy*qz - 2*qx*qw);
	w_orient.a_world_z = telem.ax * (2*qx*qz - 2*qy*qw) + telem.ay * (2*qy*qz + 2*qx*qw) + telem.az * (1 - 2*qx*qx - 2*qy*qy) - 9.81;
}

const Vector<double>& MadgwickFilter::get_quaternion() const { return q; }
