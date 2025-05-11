#ifndef IMU_FILTER_H
#define IMU_FILTER_H

#include "../data_structures.h"

class IMUFilter {
public:
    virtual void fuse_data(Telemetry& telem, double dt) = 0;
    virtual void update_orientation(Orientation& orient) = 0;
    virtual void update_world_orientation(Telemetry& telem, Orientation orient, WorldOrientation& w_orient) = 0;
    virtual ~IMUFilter() = default;
};

#endif // IMU_FILTER_H
