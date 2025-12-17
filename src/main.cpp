#include "main.h"
#include "copro/Coprocessor.hpp"
#include "copro/OTOS.hpp"

void initialize() {
    pros::delay(3000);
    std::cout << "Starting!" << std::endl;
    copro::init(21, 921600);
    std::cout << "success!" << std::endl;

    while (true) {
        auto accel = otos::get_acceleration();
        std::cout << "accel2: " << accel.x << std::endl;
        pros::delay(1000);
    }
}
