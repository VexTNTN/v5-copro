#pragma once

#include <cstdint>
namespace otos {
struct Pose {
  float x;
  float y;
  float h;
};

struct Status {
  bool warn_tilt_angle;
  bool warn_optical_tracking;
  bool optical_fatal;
  bool imu_fatal;
  bool pros_error;
};

Status getStatus() noexcept;

int selfTest() noexcept;

int resetTracking() noexcept;

//////////////////////////////////////
// pose
/////////////////

Pose get_pose() noexcept;

int set_pose(Pose pose) noexcept;

//////////////////////////////////////
// offset
/////////////////

int set_offset(Pose pose) noexcept;

//////////////////////////////////////
// linear scalar
/////////////////

float get_linear_scalar() noexcept;

int set_linear_scalar(float scalar) noexcept;

//////////////////////////////////////
// angular scalar
/////////////////

float get_angular_scalar() noexcept;

int set_angular_scalar(float scalar) noexcept;

//////////////////////////////////////
// calibrate
/////////////////

int calibrate(uint8_t samples) noexcept;

int isCalibrated() noexcept;
} // namespace otos