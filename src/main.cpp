#include "main.h"
#include "copro/Coprocessor.hpp"
#include <cstring>
#include <system_error>

std::string deserialize(const std::vector<uint8_t> &vec) {
  // Interpret the vector's data as a C-string and construct a string
  return std::string(reinterpret_cast<const char *>(vec.data()), vec.size());
}

static std::vector<uint8_t> serialize(std::string s) {
  std::vector<uint8_t> rtn;
  for (uint8_t b : s)
    rtn.push_back(b);
  return rtn;
}

void initialize() {
  pros::lcd::initialize();
  /*
    pros::delay(1000);
  pros::Serial s(21, 921600);
  while (true) {
    int r = s.read_byte();
    if (r == -1 || r == PROS_ERR)
      continue;
    else {
      std::cout << int(r) << std::endl;
    }
  }
  */

  copro::Coprocessor co(21, 921600);
  /*
  pros::delay(3000);
  std::vector<uint8_t> t({0xBB, 0xBB, 0xBB, 0xAA, 0x55});
  while (true) {
    co.write(t);
    pros::delay(5);
  }
  */

  while (true) {
    try {
      co.read();
      std::cout << "success" << std::endl;
    } catch (std::system_error e) {
      if (e.code().value() == ENODATA)
        continue;
      std::cout << e.code().value() << " " << e.what() << std::endl;
    }
    pros::delay(10);
  }
  /*
  while (true) {
    std::vector<uint8_t> msg;
    try {
      msg = co.read();
      pros::lcd::print(0, vectorToString(msg).c_str());
    } catch (std::system_error e) {
      pros::lcd::print(1, "%d", e.code().value());
      pros::lcd::print(2, "time: %d", pros::millis());
      if (e.code().value() != ENODATA) {
        std::cout << e.code().value() << std::endl;
        break;
      }
    }
    pros::delay(10);
    // break;
  }
  */

  // read and parse data
  // auto data = read(serial);
  // std::cout << "read " << data.size() << std::endl;
  // std::cout << int(data[0]) << std::endl;
  // std::cout << std::string(reinterpret_cast<const char*>(data.data()),
  // data.size()) << std::endl;

  // wait 5 seconds
  // pros::delay(5000);

  // send data
  // serial.write(stringToBuf("Hi!"), 2);
}