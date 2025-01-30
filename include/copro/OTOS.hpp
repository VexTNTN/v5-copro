#pragma once

#include <cstdint>
namespace otos {
struct Pose {
  float x;
  float y;
  float h;
};

int getStatus() noexcept;

int selfTest() noexcept;

int resetTracking() noexcept;

//////////////////////////////////////
// pose
/////////////////

Pose get_pose() noexcept;

int set_pose(Pose pose) noexcept;

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