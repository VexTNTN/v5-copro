#include "main.h"
#include "pros/serial.hpp"
#include <cstring>

void initialize() {
    // set up serial port
    pros::lcd::initialize();
    pros::Serial serial(21);
    serial.set_baudrate(921600);
    pros::delay(1000); // delay just in case

    // read and parse data
    //auto data = read(serial);
    //std::cout << "read " << data.size() << std::endl;
    //std::cout << int(data[0]) << std::endl;
    //std::cout << std::string(reinterpret_cast<const char*>(data.data()), data.size()) << std::endl;
    
    // wait 5 seconds
    // pros::delay(5000);

    // send data
    // serial.write(stringToBuf("Hi!"), 2);
}