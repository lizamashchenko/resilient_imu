#ifndef DATA_STRUCTURES_H
#define DATA_STRUCTURES_H

struct Vector3D {
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

struct WorldOrientation {
  double a_world_x = 0, a_world_y = 0, a_world_z = 0;
};

#endif //DATA_STRUCTURES_H
