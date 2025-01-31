#include "main.h"
#include "copro/Coprocessor.hpp"
#include "copro/OTOS.hpp"
#include <cstring>

void initialize() {
  pros::lcd::initialize();
  copro::init(13, 921600);
  pros::delay(2000);
  std::cout << "initialized" << std::endl;

  // calibrate
  std::cout << "calibrating..." << otos::calibrate(255) << std::endl;
  pros::delay(3000);
  std::cout << "calibrated: " << otos::isCalibrated() << std::endl;
  pros::delay(1);

  // set linear scalar
  std::cout << "setting linear scalar: " << otos::set_linear_scalar(1.1)
            << std::endl;
  pros::delay(1);
  std::cout << "linear scalar: " << otos::get_linear_scalar() << std::endl;
  pros::delay(1);

  // set angular scalar
  std::cout << "setting angular scalar: " << otos::set_angular_scalar(1.1)
            << std::endl;
  pros::delay(1);
  std::cout << "angular scalar: " << otos::get_linear_scalar() << std::endl;
  pros::delay(1);

  // set offset
  std::cout << "setting offset: " << otos::set_offset({0, 0, 0}) << std::endl;
  pros::delay(1);

  // reset tracking
  std::cout << "resetting tracking: " << otos::resetTracking() << std::endl;
  pros::delay(100);

  // 0, 0, 0 -> 0.06, 0, 0
  // 1, 1, 1 ->

  // get status
  std::cout << "getting status... " << std::endl;
  otos::Status status = otos::getStatus();
  std::cout << "tilt angle warning: " << status.warn_tilt_angle << std::endl;
  std::cout << "optical tracking warning: " << status.warn_optical_tracking
            << std::endl;
  std::cout << "fatal optical error: " << status.optical_fatal << std::endl;
  std::cout << "fatal imu error: " << status.imu_fatal << std::endl;
  std::cout << "pros error: " << status.pros_error << std::endl;
  pros::delay(1);

  std::cout << "setting pose: " << otos::set_pose({10, 10, 10}) << std::endl;
  pros::delay(100);

  std::cout << "getting pose: " << std::endl;

  while (true) {
    otos::Pose pose = otos::get_pose();
    std::cout << "x: " << pose.x << ", y: " << pose.y << ", h: " << pose.h
              << std::endl;
    pros::delay(10);
  }
}