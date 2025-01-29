#include "main.h"
#include "copro/Coprocessor.hpp"
#include <cstring>
#include <system_error>

void initialize() {
  pros::lcd::initialize();
  copro::init(21, 921600);

  while (true) {
    int start = pros::millis();
    /*
    co.write(serialize("AAAA"));
    bool run = true;
    while (run) {
      try {
        co.read();
        run = false;
      } catch (std::system_error e) {
        if (e.code().value() != ENODATA) {
          std::cout << e.code() << std::endl;
        }
      }
    }
    std::cout << "ping" << pros::millis() - start << std::endl;

    pros::delay(10);
    */
  }
}