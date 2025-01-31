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
}