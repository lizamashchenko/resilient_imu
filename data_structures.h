//
// Created by liza on 09.04.25.
//

#ifndef DATA_STRUCTURES_H
#define DATA_STRUCTURES_H

struct Relative_Position {
  double x = 0, y = 0, z = 0;
};

struct Telemetry {
  double ax = 0, ay = 0, az = 0;
  double gx = 0, gy = 0, gz = 0;
  // double mx, my, mz;
};

struct Orientation {
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
};

#endif //DATA_STRUCTURES_H
