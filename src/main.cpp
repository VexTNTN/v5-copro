#include "main.h"
#include "copro/Coprocessor.hpp"
#include "copro/OTOS.hpp"
#include "pros/apix.h" // IWYU pragma: keep

copro::Coprocessor coprocessor(21);
copro::Otos otos(coprocessor);

void initialize() {
    // uncomment to disable stream multiplexing and COBS, so cargo-v5 can read
    // the terminal
    // pros::c::serctl(SERCTL_DISABLE_COBS, NULL);
    // pros::c::serctl(SERCTL_DEACTIVATE, (void*)0x74756f73);
    // and don't forget to set USE_PACKAGE to 0 in the Makefile!

    // delay to allow time for serial terminal to connect
    // pros::delay(3000);
    std::cout << "Starting!" << std::endl;

    // initialize comms with coprocessor (blocking)
    auto start = pros::micros();

    coprocessor.init();
    std::cout << "success!" << std::endl;
    printf("init finished in: %.3f\n", (pros::micros() - start) / 1000.0f);
    start = pros::micros();

    otos.set_linear_scalar(0.9);
    printf("set linear finished in: %.3f\n",
           (pros::micros() - start) / 1000.0f);
    start = pros::micros();

    otos.set_angular_scalar(0.9);
    printf("set angular finished in: %.3f\n",
           (pros::micros() - start) / 1000.0f);
    start = pros::micros();

    otos.calibrate(255);
    printf("calibrate finished in: %.3f\n", (pros::micros() - start) / 1000.0f);

    pros::delay(1000);
    start = pros::micros();

    otos.reset_tracking();
    printf("reset finished in: %.3f\n", (pros::micros() - start) / 1000.0f);
    start = pros::micros();

    otos.set_pose({ 0, 0, 0 });
    printf("set_pose finished in: %.3f\n", (pros::micros() - start) / 1000.0f);
    start = pros::micros();

    auto p = otos.get_pose();
    printf("get_pose finished in: %.3f\n", (pros::micros() - start) / 1000.0f);
    start = pros::micros();

    printf("is calibrated: %d\n", otos.is_calibrated().value());
    printf("is calibrated finished in: %.3f\n",
           (pros::micros() - start) / 1000.0f);
    start = pros::micros();
    printf("AS: %.3f, LS: %.3f\n",
           otos.get_angular_scalar().value(),
           otos.get_linear_scalar().value());

    auto a = otos.get_acceleration();
    printf("get_acceleration: x: %.2f, y: %.2f, h: %.2f\n", a->x, a->y, a->h);
    printf("get_acceleration finished in: %.3f\n",
           (pros::micros() - start) / 1000.0f);
    start = pros::micros();

    otos.set_offset({ 0, 0, 0 });

    // print acceleration data along the x axis every second
    while (true) {
        auto p = otos.get_pose();
        printf("x: %.2f, y: %.2f, h: %.2f\n", p->x, p->y, p->h);
        // auto a = otos::get_acceleration();
        // printf("a: x: %6.2f, y: %6.2f, h: %6.2f\n", a.x, a.y, a.h);
        pros::delay(50);
    }
}
