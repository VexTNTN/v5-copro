#include "main.h"
#include "copro/Coprocessor.hpp"
#include "copro/OTOS.hpp"
#include "pros/llemu.hpp"
#include <cstring>

// status is 4 when driver control

void initialize() {

  pros::lcd::initialize();
  pros::lcd::print(0, "initializing: %d", copro::init(13, 921600));
  pros::lcd::print(1, "errno: %d", errno);
  pros::delay(10);
  while (true) {
    auto a = otos::get_pose();
    std::cout << "x: " << a.x << ", y: " << a.y << ", h: " << a.h << std::endl;
    pros::delay(10);
  }
}