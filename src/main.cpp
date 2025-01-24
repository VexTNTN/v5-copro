#include "main.h"
#include "pros/serial.hpp"

void initialize() {
    pros::lcd::initialize();
    pros::Serial serial(21);
    serial.set_baudrate(9600);
    pros::delay(1000);

    while (true) {
        if (serial.get_read_avail() == 0) {
            pros::delay(10);
            continue;
        }

        std::uint8_t buf[200];
        serial.read(buf, 200);

        std::ostringstream convert;
        for (int a = 0; a < 200; a++) {
            convert << (int)buf[a];
        }

        std::string str = convert.str();

        pros::lcd::print(0, str.c_str());

        pros::delay(10);
    }
}