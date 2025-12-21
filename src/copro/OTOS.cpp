#include "OTOS.hpp"
#include "Coprocessor.hpp"
#include "pros/error.h"
#include "pros/rtos.hpp"
#include <cmath>

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

// otos::getAcceleration,         // 12 // in progress

// otos::getAccelerationStdDev,   // 13 // low prio
// otos::getLinearUnit,           // 14 // unnecessary
// otos::setLinearUnit,           // 15 // unnecessary
// otos::getAngularUnit,          // 16 // unnecessary
// otos::setAngularUnit,          // 17 // unnecessary
// otos::getLinearScalar,         // 18 // done
// otos::setLinearScalar,         // 19 // done
// otos::getAngularScalar,        // 20 // done
// otos::setAngularScalar,        // 21 // done
// otos::getSignalProcessConfig,  // 22 // low prio
// otos::setSignalProcessConfig,  // 23 // low prio
// otos::selfTest,                // 24 // done
// otos::calibrate,               // 25 // done
// otos::isCalibrated,            // 26 // done
// otos::getOffset,               // 27 // impossible until FW bug fixed
// otos::setOffset                // 28 // done

// TODO: add mutex for copro communication?

namespace otos {

//////////////////////////////////////
// constants
/////////////////
constexpr int READ_TIMEOUT = 20;

constexpr float kRadianToDegree = 180.0 / 3.14159;
constexpr float kDegreeToRadian = 3.14159 / 180.0;
constexpr float kMeterToInch = 39.3701;
constexpr float kInchToMeter = 1.0 / kMeterToInch;

constexpr float kMeterToInt16 = 32768.0 / 10.0;
constexpr float kInt16ToMeter = 1.0 / kMeterToInt16;
constexpr float kInt16ToInch = kInt16ToMeter * kMeterToInch;
constexpr float kInchToInt16 = 1.0 / kInt16ToInch;

constexpr float kRadToInt16 = 32768.0 / 3.14159;
constexpr float kInt16ToRad = 1.0 / kRadToInt16;
constexpr float kInt16ToDeg = kInt16ToRad * kRadianToDegree;
constexpr float kDegToInt16 = 1.0 / kInt16ToDeg;

static constexpr float kMpssToInt16 = 32768.0f / (16.0f * 9.80665f);
static constexpr float kInt16ToMpss = 1.0f / kMpssToInt16;

static constexpr float kRpssToInt16 = 32768.0f / (M_PI * 1000.0f);
static constexpr float kInt16ToRpss = 1.0f / kRpssToInt16;

// helper function to convert float to int16 with proper clamping and rounding
inline static std::int16_t to_i16(float v) noexcept {
    constexpr auto I16_MIN = std::numeric_limits<std::int16_t>::min(); // -32768
    constexpr auto I16_MAX = std::numeric_limits<std::int16_t>::max(); //  32767

    if (std::isnan(v)) return I16_MAX;
    if (std::isinf(v)) return std::signbit(v) ? I16_MIN : I16_MAX;

    // Clamp first to avoid any overflow/FE_INVALID issues in rounding routines
    double d = std::clamp(static_cast<double>(v),
                          static_cast<double>(I16_MIN),
                          static_cast<double>(I16_MAX));

    // std::round: half away from zero
    long long r = static_cast<long long>(std::round(d));

    // After the clamp, r is guaranteed in-range, so this cast is safe
    return static_cast<std::int16_t>(r);
}

// helper function to convert float to int8 with proper clamping and rounding
inline static std::int8_t to_i8(float v) noexcept {
    constexpr std::int8_t I8_MIN = -128;
    constexpr std::int8_t I8_MAX = 127;

    if (std::isnan(v)) return I8_MAX;
    if (std::isinf(v)) return std::signbit(v) ? I8_MIN : I8_MAX;

    // saturate then round (half away from zero)
    double d =
      std::clamp(static_cast<double>(v), double(I8_MIN), double(I8_MAX));
    long long r = static_cast<long long>(std::round(d));
    return static_cast<std::int8_t>(r);
}

Status getStatus() noexcept {
    constexpr int ID = 1;

    union {
        struct {
            uint8_t warn_tilt_angle       : 1;
            uint8_t warn_optical_tracking : 1;
            uint8_t reserved              : 4;
            uint8_t optical_fatal         : 1;
            uint8_t imu_fatal             : 1;
        };

        uint8_t value;
    } s;

    auto raw = copro::write_and_receive(ID, {}, READ_TIMEOUT);
    if (raw.size() != 1) {
        return { 0, 0, 0, 0, 1 };
    } else {
        s.value = raw.at(0);
        return { static_cast<bool>(s.warn_tilt_angle),
                 static_cast<bool>(s.warn_optical_tracking),
                 static_cast<bool>(s.optical_fatal),
                 static_cast<bool>(s.imu_fatal),
                 0 };
    }
}

int selfTest() noexcept {
    constexpr int ID = 24;
    const auto raw = copro::write_and_receive(ID, {}, READ_TIMEOUT);
    if (raw.size() != 1) {
        return PROS_ERR;
    } else {
        return 1;
    }
}

int resetTracking() noexcept {
    constexpr int ID = 3;
    const auto raw = copro::write_and_receive(ID, {}, READ_TIMEOUT);
    if (raw.size() != 1) {
        return PROS_ERR;
    } else {
        return 1;
    }
}

//////////////////////////////////////
// pose
/////////////////

Pose get_pose() noexcept {
    constexpr int ID = 7;
    constexpr Pose ERROR = { std::numeric_limits<float>::infinity(),
                             std::numeric_limits<float>::infinity(),
                             std::numeric_limits<float>::infinity() };

    // request, recieve
    const auto tmp = copro::write_and_receive(ID, {}, READ_TIMEOUT);
    if (tmp.size() != 6) {
        return ERROR;
    }

    // parse raw data
    int16_t rawX = (tmp[1] << 8) | tmp[0];
    int16_t rawY = (tmp[3] << 8) | tmp[2];
    int16_t rawH = (tmp[5] << 8) | tmp[4];

    return { rawX * kInt16ToInch, rawY * kInt16ToInch, rawH * kInt16ToDeg };
}

int set_pose(Pose pose) noexcept {
    constexpr int ID = 8;
    // cast
    int16_t rawX = to_i16(pose.x * kInchToInt16);
    int16_t rawY = to_i16(pose.y * kInchToInt16);
    int16_t rawH = to_i16(pose.h * kDegToInt16);
    // init vector
    std::vector<uint8_t> out(6);
    // serialize
    out.at(0) = rawX & 0xFF;
    out.at(1) = (rawX >> 8) & 0xFF;
    out.at(2) = rawY & 0xFF;
    out.at(3) = (rawY >> 8) & 0xFF;
    out.at(4) = rawH & 0xFF;
    out.at(5) = (rawH >> 8) & 0xFF;

    // write and get response
    auto raw = copro::write_and_receive(ID, out, READ_TIMEOUT);
    pros::delay(10);
    if (raw.size() != 1) {
        return PROS_ERR;
    }

    // check that the pose was actually set
    auto p = get_pose();
    // check for error
    if (p.x == INFINITY) {
        return PROS_ERR;
    }

    // check that all the fields are the same
    if (std::fabs(p.x - pose.x) > 1) {
        return PROS_ERR;
    }

    if (std::fabs(p.y - pose.y) > 1) {
        return PROS_ERR;
    }

    if (std::fabs(p.h - pose.h) > 1) {
        return PROS_ERR;
    }

    // success
    return 1;
}

//////////////////////////////////////
// acceleration
/////////////////

Acceleration get_acceleration() noexcept {
    constexpr int ID = 12;
    constexpr Pose ERROR = { std::numeric_limits<float>::infinity(),
                             std::numeric_limits<float>::infinity(),
                             std::numeric_limits<float>::infinity() };

    // request, recieve
    const auto tmp = copro::write_and_receive(ID, {}, READ_TIMEOUT);
    if (tmp.size() != 6) {
        return ERROR;
    }

    // parse raw data
    int16_t rawX = (tmp[1] << 8) | tmp[0];
    int16_t rawY = (tmp[3] << 8) | tmp[2];
    int16_t rawH = (tmp[5] << 8) | tmp[4];

    return { rawX * kInt16ToMpss * kMeterToInch,
             rawY * kInt16ToMpss * kMeterToInch,
             static_cast<float>(rawH * kInt16ToRpss / 360.0) };
}

//////////////////////////////////////
// offset
/////////////////

int set_offset(Pose pose) noexcept {
    constexpr int ID = 28;
    // cast
    int16_t rawX = to_i16(pose.x * kInchToInt16);
    int16_t rawY = to_i16(pose.y * kInchToInt16);
    int16_t rawH = to_i16(pose.h * kDegToInt16);
    // init vector
    std::vector<uint8_t> out(6, 0);
    // serialize
    out.at(0) = rawX & 0xFF;
    out.at(1) = (rawX >> 8) & 0xFF;
    out.at(2) = rawY & 0xFF;
    out.at(3) = (rawY >> 8) & 0xFF;
    out.at(4) = rawH & 0xFF;
    out.at(5) = (rawH >> 8) & 0xFF;

    // write and get response
    const auto raw = copro::write_and_receive(ID, out, READ_TIMEOUT);
    if (raw.size() != 1) {
        return PROS_ERR;
    } else {
        return 1;
    }
}

//////////////////////////////////////
// linear scalar
/////////////////

float get_linear_scalar() noexcept {
    constexpr int ID = 18;
    const auto raw = copro::write_and_receive(ID, {}, READ_TIMEOUT);

    if (raw.size() != 1) {
        return std::numeric_limits<float>::infinity();
    }

    return 0.001f * static_cast<int8_t>(raw.at(0)) + 1.0f;
}

int set_linear_scalar(float scalar) noexcept {
    constexpr int ID = 19;

    const float scaled = (scalar - 1.0f) * 1000.0f;
    const std::int8_t raw_i8 = to_i8(scaled);
    const std::uint8_t raw = std::bit_cast<std::uint8_t>(raw_i8);

    const auto err = copro::write_and_receive(ID, { raw }, READ_TIMEOUT);
    if (err.size() != 1) return PROS_ERR;

    const auto s = get_linear_scalar();
    if (!std::isfinite(s)) return PROS_ERR; // stronger than s == INFINITY
    if (std::abs(s - scalar) > 0.02f) return PROS_ERR;
    return 1;
}

//////////////////////////////////////
// angular scalar
/////////////////

float get_angular_scalar() noexcept {
    constexpr int ID = 20;

    const auto raw = copro::write_and_receive(ID, {}, READ_TIMEOUT);
    if (raw.size() != 1) {
        return std::numeric_limits<float>::infinity();
    }

    return 0.001f * static_cast<int8_t>(raw.at(0)) + 1.0f;
}

int set_angular_scalar(float scalar) noexcept {
    constexpr int ID = 21;

    const float scaled = (scalar - 1.0f) * 1000.0f;
    const std::int8_t raw_i8 = to_i8(scaled);
    const std::uint8_t raw = std::bit_cast<std::uint8_t>(raw_i8);

    const auto err = copro::write_and_receive(ID, { raw }, READ_TIMEOUT);
    if (err.size() != 1) return PROS_ERR;

    // check that the angular scalar was actually set
    const auto s = get_angular_scalar();
    if (!std::isfinite(s)) return PROS_ERR; // catches +inf, -inf, NaN
    if (std::abs(s - scalar) > 0.02f) return PROS_ERR;

    return 1;
}

//////////////////////////////////////
// calibrate
/////////////////

int calibrate(uint8_t samples) noexcept {
    constexpr int ID = 25;

    const auto err = copro::write_and_receive(ID, { samples }, READ_TIMEOUT);
    if (err.size() != 1) {
        return PROS_ERR;
    }

    return 1;
}

int isCalibrated() noexcept {
    constexpr int ID = 26;

    const auto err = copro::write_and_receive(ID, {}, READ_TIMEOUT);

    if (err.size() != 1) {
        return PROS_ERR;
    }

    return err.at(0);
}

} // namespace otos
