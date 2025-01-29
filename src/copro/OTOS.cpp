#include "copro/OTOS.hpp"
#include "copro/Coprocessor.hpp"
#include "pros/rtos.hpp"
#include <iostream>

namespace otos {

//////////////////////////////////////
// constants
/////////////////
constexpr int READ_TIMEOUT = 10;

constexpr float RADIAN_TO_DEGREE = 180.0 / 3.14159;
constexpr float DEGREE_TO_RADIAN = 3.14159 / 180.0;

constexpr float RAD_TO_INT16 = 32768.0 / 3.14159;
constexpr float INT16_TO_RAD = 1.0 / RAD_TO_INT16;
constexpr float INT16_TO_DEG = INT16_TO_RAD * RADIAN_TO_DEGREE;
constexpr float DEG_TO_INT16 = 1.0 / INT16_TO_DEG;

constexpr float METER_TO_INCH = 39.3701;
constexpr float INCH_TO_METER = 1.0 / METER_TO_INCH;

constexpr float METER_TO_INT16 = 32768.0 / 10.0;
constexpr float INT16_TO_METER = 1.0 / METER_TO_INT16;

constexpr float INT16_TO_INCH = INT16_TO_METER * METER_TO_INCH;
constexpr float INCH_TO_INT16 = 1.0 / INT16_TO_INCH;

//////////////////////////////////////
// util
/////////////////

static std::vector<uint8_t>
write_and_receive(uint8_t id, const std::vector<uint8_t> &data, int timeout) {
  // prepare data
  std::vector<uint8_t> out(data.size() + 1);
  out.push_back(id);
  out.insert(out.begin() + 1, data.begin(), data.end());
  // write data
  copro::write(out);
  // wait for a response
  const int start = pros::millis();
  while (true) {
    try {
      return copro::read();
    } catch (std::system_error &e) {
      if (e.code().value() == ENODATA) {
        if (pros::millis() - start > timeout) {
          throw e;
        } else {
          continue;
        }
      } else {
        throw e;
      }
    }
  }
}

template <typename T> static std::vector<uint8_t> serialize(const T &data) {
  static_assert(std::is_trivially_copyable_v<T>,
                "Type must be trivially copyable");
  auto raw = std::bit_cast<std::array<uint8_t, sizeof(T)>>(data);
  std::vector<uint8_t> out(sizeof(T));
  for (int i = 0; i < sizeof(T); ++i) {
    out.at(i) = raw.at(i);
  }
  return out;
}

template <typename T, int N>
static T deserialize(const std::vector<uint8_t> &data) {
  static_assert(std::is_trivially_copyable_v<T>,
                "Type must be trivially copyable");
  std::array<uint8_t, N> raw;
  for (int i = 0; i < N; ++i) {
    raw.at(i) = data.at(i);
  }
  return std::bit_cast<T>(raw);
}

//////////////////////////////////////
// OTOS
/////////////////

int getStatus() {
  constexpr int ID = 1;
  try {
    return static_cast<int>(write_and_receive(ID, {}, READ_TIMEOUT).at(0));
  } catch (std::system_error &e) {
    std::cout << e.code() << ", " << e.what() << std::endl;
    return 1;
  }
}

int selfTest() {
  constexpr int ID = 24;
  try {
    return static_cast<int>(write_and_receive(ID, {}, READ_TIMEOUT).at(0));
  } catch (std::system_error &e) {
    std::cout << e.code() << ", " << e.what() << std::endl;
    return 1;
  }
}

int resetTracking() {
  constexpr int ID = 3;
  try {
    return static_cast<int>(write_and_receive(ID, {}, READ_TIMEOUT).at(0));
  } catch (std::system_error &e) {
    std::cout << e.code() << ", " << e.what() << std::endl;
    return 1;
  }
}

//////////////////////////////////////
// pose
/////////////////

Pose get_pose() {
  constexpr int ID = 7;
  constexpr Pose ERROR = {std::numeric_limits<float>::infinity(),
                          std::numeric_limits<float>::infinity(),
                          std::numeric_limits<float>::infinity()};

  std::vector<uint8_t> raw(6);
  // request, receive
  try {
    raw = write_and_receive(ID, {}, READ_TIMEOUT);
  } catch (std::system_error &e) {
    std::cout << e.code() << ", " << e.what() << std::endl;
    return ERROR;
  }

  // error checking
  bool err = true;
  for (uint8_t b : raw) {
    if (b != 1) {
      err = false;
    }
  }
  if (err) {
    return ERROR;
  }

  // parse raw data
  struct RawPose {
    int16_t x;
    int16_t y;
    int16_t h;
  } raw_pose;

  raw_pose = deserialize<RawPose, 6>(raw);

  return {raw_pose.x * INT16_TO_INCH, raw_pose.y * INT16_TO_INCH,
          raw_pose.h * INT16_TO_DEG};
}

int set_pose(Pose pose) {
  constexpr int ID = 8;
  // cast
  int rawX = static_cast<int>(pose.x * INCH_TO_INT16);
  int rawY = static_cast<int>(pose.y * INCH_TO_INT16);
  int rawH = static_cast<int>(pose.h * DEG_TO_INT16);
  // init vector
  std::vector<uint8_t> out(6);
  // serialize
  out.at(0) = rawX & 0xFF;
  out.at(1) = (rawX >> 8) & 0xFF;
  out.at(2) = rawY & 0xFF;
  out.at(3) = (rawY >> 8) & 0xFF;
  out.at(4) = rawH * 0xFF;
  out.at(5) = (rawH >> 8) & 0xFF;
  // write and get response
  return write_and_receive(8, out, READ_TIMEOUT).at(0);
}

} // namespace otos