#include "main.h"
#include "copro/Coprocessor.hpp"
#include "copro/OTOS.hpp"
#include <cstring>

void initialize() {
  pros::lcd::initialize();
  copro::init(13, 921600);

  pros::delay(2000);
  std::cout << "initialized" << std::endl;

  std::cout << "calibrating..." << otos::calibrate(10) << std::endl;
  pros::delay(3000);
  std::cout << "calibrated: " << otos::isCalibrated() << std::endl;

  pros::delay(1000);

  std::cout << otos::set_angular_scalar(1.1) << std::endl;
  pros::delay(1);
  std::cout << otos::get_angular_scalar() << std::endl;
}