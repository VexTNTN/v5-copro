#include "main.h"
#include "copro/Coprocessor.hpp"
#include "copro/OTOS.hpp"
#include "pros/apix.h" // IWYU pragma: keep

void initialize() {
    // uncomment to disable stream multiplexing and COBS, so cargo-v5 can read
    // the terminal
    // pros::c::serctl(SERCTL_DISABLE_COBS, NULL);
    // pros::c::serctl(SERCTL_DEACTIVATE, (void*)0x74756f73);
    // and don't forget to set USE_PACKAGE to 0 in the Makefile!

    // delay to allow time for serial terminal to connect
    pros::delay(3000);
    std::cout << "Starting!" << std::endl;

    // initialize comms with coprocessor (blocking)
    copro::init(21, 921600);
    std::cout << "success!" << std::endl;

    // print acceleration data along the x axis every second
    while (true) {
        auto accel = otos::get_acceleration();
        std::cout << "accel2: " << accel.x << std::endl;
        pros::delay(1000);
    }
}
