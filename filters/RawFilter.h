#ifndef RAWFILTER_H
#define RAWFILTER_H
#include <math.h>

#include "IMUFilter.h"


class RawFilter : public IMUFilter {
private:
    double angle_x = 0.0; // Roll
    double angle_y = 0.0; // Pitch
    double angle_z = 0.0; // Yaw

public:
    RawFilter() = default;

    void fuse_data(Telemetry& telem, double dt) override;
    void update_orientation(Orientation &orient) override;
    void update_world_orientation(Telemetry &telem, Orientation orient, WorldOrientation &w_orient) override;
};




#endif //RAWFILTER_H
