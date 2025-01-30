#include "copro/OTOS.hpp"
#include "copro/Coprocessor.hpp"
#include "pros/error.h"
#include "pros/rtos.hpp"
#include <iostream>
#include <system_error>

// getCoprocessorVersion,         // 0 // low prio
// otos::getStatus,               // 1 // done
// otos::getVersion,              // 2 // low prio
// otos::resetTracking,           // 3 // done
// otos::getPosVelAccel,          // 4 // low prio
// otos::getPosVelAccelStdDev,    // 5 // low prio
// otos::getPosVelAccelAndStdDev, // 6 // low prio
// otos::getPose,                 // 7  // done
// otos::setPose,                 // 8  // done
// otos::getPositionStdDev,       // 9  // low prio
// otos::getVelocity,             // 10 // low prio
// otos::getVelocityStdDev,       // 11 // low prio
// otos::getAcceleration,         // 12 // low prio
// otos::getAccelerationStdDev,   // 13 // low prio
// otos::getLinearUnit,           // 14 // low prio
// otos::setLinearUnit,           // 15 // low prio
// otos::getAngularUnit,          // 16 // low prio
// otos::setAngularUnit,          // 17 // low prio
// otos::getLinearScalar,         // 18 // done
// otos::setLinearScalar,         // 19 // done
// otos::getAngularScalar,        // 20 // done
// otos::setAngularScalar,        // 21 // done
// otos::getSignalProcessConfig,  // 22 // low prio
// otos::setSignalProcessConfig,  // 23 // low prio
// otos::selfTest,                // 24 // done
// otos::calibrate,               // 25 // done
// otos::isCalibrated,            // 26 // hi prio
// otos::getOffset,               // 27 // lo prio
// otos::setOffset                // 28 // hi prio

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

static std::vector<uint8_t> write_and_receive(uint8_t id,
                                              const std::vector<uint8_t> &data,
                                              int timeout) noexcept {
  // prepare data
  std::vector<uint8_t> out;
  out.push_back(id);
  out.insert(out.end(), data.begin(), data.end());
  // write data
  copro::write(out);
  // wait for a response
  const int start = pros::millis();
  while (true) {
    try {
      auto raw = copro::read();
      std::vector<uint8_t> rtn;
      rtn.insert(rtn.end(), raw.begin() + 1, raw.end());
      return rtn;
    } catch (std::system_error &e) {
      if (e.code().value() == ENODATA) {
        if (pros::millis() - start > timeout) {
          std::cout << e.code() << ", " << e.what() << std::endl;
          errno = e.code().value();
          return {};
        } else {
          continue;
        }
      } else {
        std::cout << e.code() << ", " << e.what() << std::endl;
        errno = e.code().value();
        return {};
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

int getStatus() noexcept {
  constexpr int ID = 1;
  auto raw = write_and_receive(ID, {}, READ_TIMEOUT);
  if (raw.empty()) {
    return 1;
  } else {
    return static_cast<int>(raw.at(0));
  }
}

int selfTest() noexcept {
  constexpr int ID = 24;
  const auto raw = write_and_receive(ID, {}, READ_TIMEOUT);
  if (raw.empty()) {
    return PROS_ERR;
  } else {
    return static_cast<int>(raw.at(0));
  }
}

int resetTracking() noexcept {
  constexpr int ID = 3;
  auto raw = write_and_receive(ID, {}, READ_TIMEOUT);
  if (raw.empty()) {
    return PROS_ERR;
  } else {
    return static_cast<int>(raw.at(0));
  }
}

//////////////////////////////////////
// pose
/////////////////

Pose get_pose() noexcept {
  constexpr int ID = 7;
  constexpr Pose ERROR = {std::numeric_limits<float>::infinity(),
                          std::numeric_limits<float>::infinity(),
                          std::numeric_limits<float>::infinity()};

  // request, receive
  auto tmp = write_and_receive(ID, {}, READ_TIMEOUT);
  if (tmp.empty()) {
    return ERROR;
  }

  // error checking
  bool err = true;
  for (uint8_t b : tmp) {
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

  raw_pose = deserialize<RawPose, 6>(tmp);

  return {raw_pose.x * INT16_TO_INCH, raw_pose.y * INT16_TO_INCH,
          raw_pose.h * INT16_TO_DEG};
}

int set_pose(Pose pose) noexcept {
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
  auto raw = write_and_receive(8, out, READ_TIMEOUT);
  if (raw.empty()) {
    return PROS_ERR;
  } else {
    return static_cast<int>(raw.at(0));
  }
}

//////////////////////////////////////
// linear scalar
/////////////////

float get_linear_scalar() noexcept {
  constexpr int ID = 18;
  auto raw = write_and_receive(ID, {}, READ_TIMEOUT);
  if (raw.empty()) {
    return std::numeric_limits<float>::infinity();
  } else if (raw.at(0) == 0) {
    return std::numeric_limits<float>::infinity();
  } else {
    return 0.001f * static_cast<int8_t>(raw.at(0)) + 1.0f;
  }
}

int set_linear_scalar(float scalar) noexcept {
  constexpr int ID = 19;
  auto raw =
      static_cast<uint8_t>(static_cast<int8_t>(scalar - 1.0f) * 1000 + 0.5f);
  auto err = write_and_receive(19, {raw}, READ_TIMEOUT);
  if (err.empty()) {
    return PROS_ERR;
  } else {
    return 0;
  }
}

//////////////////////////////////////
// angular scalar
/////////////////

float get_angular_scalar() noexcept {
  constexpr int ID = 20;
  auto raw = write_and_receive(ID, {}, READ_TIMEOUT);
  if (raw.empty()) {
    return std::numeric_limits<float>::infinity();
  } else if (raw.at(0) == 0) {
    return std::numeric_limits<float>::infinity();
  } else {
    return 0.001f * static_cast<int8_t>(raw.at(0)) + 1.0f;
  }
}

int set_angular_scalar(float scalar) noexcept {
  constexpr int ID = 21;
  auto raw =
      static_cast<uint8_t>(static_cast<int8_t>(scalar - 1.0f) * 1000 + 0.5f);
  auto err = write_and_receive(19, {raw}, READ_TIMEOUT);
  if (err.empty()) {
    return PROS_ERR;
  } else {
    return 0;
  }
}

//////////////////////////////////////
// calibrate
/////////////////

int calibrate(uint8_t samples) noexcept {
  constexpr int ID = 25;
  auto err = write_and_receive(ID, {samples}, READ_TIMEOUT);
  if (err.empty() || (err.at(0) != 0 && err.at(0) != 1)) {
    return PROS_ERR;
  }
  return err.at(0);
}

int isCalibrated() noexcept {
  constexpr int ID = 26;
  auto err = write_and_receive(ID, {}, READ_TIMEOUT);
  if (err.empty() || (err.at(0) != 0 && err.at(0) != 1)) {
    return PROS_ERR;
  }
  return err.at(0);
}

} // namespace otos