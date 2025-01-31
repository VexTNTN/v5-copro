#include "main.h"
#include "copro/Coprocessor.hpp"
#include "copro/OTOS.hpp"
#include "pros/llemu.hpp"
#include <cstring>

// status is 4 when driver control

void initialize() {

  pros::lcd::initialize();
  std::cout << "initializing: " << copro::init(13, 921600) << std::endl;
  std::cout << "errno: " << errno << std::endl;
}