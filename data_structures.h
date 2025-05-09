//
// Created by liza on 09.04.25.
//

#ifndef DATA_STRUCTURES_H
#define DATA_STRUCTURES_H

struct Relative_Position {
  double x, y, z;
};

struct Telemetry {
  double ax, ay, az;
  double gx, gy, gz;
  // double mx, my, mz;
};

struct Orientation {
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
};

#endif //DATA_STRUCTURES_H
