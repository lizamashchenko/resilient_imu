//
// Created by liza on 11.05.25.
//

#include "RawFilter.h"

void RawFilter::fuse_data(Telemetry &telem, double dt) {
    angle_x += telem.gx * dt;
    angle_y += telem.gy * dt;
    angle_z += telem.gz * dt;
}

void RawFilter::update_orientation(Orientation &orient) {
    orient.roll = angle_x * M_PI / 180.0;
    orient.pitch = angle_y * M_PI / 180.0;
    orient.yaw = angle_z * M_PI / 180.0;
}

void RawFilter::update_world_orientation(Telemetry &telem, Orientation orient, WorldOrientation &w_orient) {
    double sr = sin(orient.roll), cr = cos(orient.roll);
    double sp = sin(orient.pitch), cp = cos(orient.pitch);
    double sy = sin(orient.yaw), cy = cos(orient.yaw);

    double R[3][3] = {
        {cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr},
        {sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr},
        {-sp,     cp * sr,                cp * cr}
    };

    w_orient.a_world_x = R[0][0] * telem.ax + R[0][1] * telem.ay + R[0][2] * telem.az;
    w_orient.a_world_y = R[1][0] * telem.ax + R[1][1] * telem.ay + R[1][2] * telem.az;
    w_orient.a_world_z = R[2][0] * telem.ax + R[2][1] * telem.ay + R[2][2] * telem.az - 9.81;
}