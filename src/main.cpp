#include "main.h"
#include "copro/Coprocessor.hpp"
#include "copro/OTOS.hpp"
#include "pros/llemu.hpp"
#include <cstring>

void initialize() {
    pros::lcd::initialize();
    // create and initialize coprocessor
    auto co = std::make_shared<copro::Coprocessor>(13, 921600);
    // co->initialize();

    // create and calibrate OTOS
    // copro::OTOS otos(co);
    // otos.calibrate();
    // while (!otos.is_calibrated()) pros::delay(10);

    // print the position measured by the otos
    while (true) {
        // auto pose = otos.get_pose();
        // std::cout << "x: " << pose.x << ", y: " << pose.y << ", h: " <<
        // pose.h << std::endl;
        pros::delay(10);
    }
}
