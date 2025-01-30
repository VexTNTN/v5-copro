#include "main.h"
#include "copro/Coprocessor.hpp"
#include "copro/OTOS.hpp"
#include <cstring>

void initialize() {
  pros::lcd::initialize();
  copro::init(15, 921600);

  pros::delay(2000);
  std::cout << "initialized" << std::endl;

  std::cout << "calibrating..." << otos::calibrate(10) << std::endl;
  pros::delay(3000);
  std::cout << "calibrated: " << otos::isCalibrated() << std::endl;

  pros::delay(1000);

  std::cout << otos::set_linear_scalar(1.1) << std::endl;
  std::cout << otos::get_linear_scalar() << std::endl;
}