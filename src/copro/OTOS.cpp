#include "copro/OTOS.hpp"
#include "Coprocessor.hpp"
#include "pros/error.h"
#include <cmath>

// otos::resetTracking,           - implemented
// otos::getPosVelAccel,          - not implemented
// otos::getPosVelAccelStdDev,    - not implemented
// otos::getPosVelAccelAndStdDev, - not implemented
// otos::getPose,                 - implemented
// otos::setPose,                 - implemented
// otos::getPositionStdDev,       - not implemented
// otos::getVelocity,             - not implemented
// otos::getVelocityStdDev,       - not implemented
// otos::getAcceleration,         - not implemented
// otos::getAccelerationStdDev,   - not implemented
// otos::getLinearScalar,         - implemented
// otos::setLinearScalar,         - implemented
// otos::getAngularScalar,        - implemented
// otos::setAngularScalar,        - implemented
// otos::getSignalProcessConfig,  - not implemented
// otos::setSignalProcessConfig,  - not implemented
// otos::calibrate,               - implemented
// otos::isCalibrated,            - implemented
// otos::getOffset,               - not implemented
// otos::setOffset                - implemented

namespace copro {
//////////////////////////////////////
// constants
/////////////////
constexpr int READ_TIMEOUT = 5;

constexpr float RADIAN_TO_DEGREE = 180.0 / 3.14159;
constexpr float METER_TO_INCH = 39.3701;

constexpr float METER_TO_INT16 = 32768.0 / 10.0;
constexpr float INT16_TO_METER = 1.0 / METER_TO_INT16;
constexpr float INT16_TO_INCH = INT16_TO_METER * METER_TO_INCH;
constexpr float INCH_TO_INT16 = 1.0 / INT16_TO_INCH;

constexpr float RAD_TO_INT16 = 32768.0 / 3.14159;
constexpr float INT16_TO_RAD = 1.0 / RAD_TO_INT16;
constexpr float INT16_TO_DEG = INT16_TO_RAD * RADIAN_TO_DEGREE;
constexpr float DEG_TO_INT16 = 1.0 / INT16_TO_DEG;

//////////////////////////////////////
// util
/////////////////

template <typename T> static std::vector<uint8_t> serialize(const T& data) {
    static_assert(std::is_trivially_copyable_v<T>, "Type must be trivially copyable");
    auto raw = std::bit_cast<std::array<uint8_t, sizeof(T)>>(data);
    std::vector<uint8_t> out(sizeof(T));
    for (int i = 0; i < sizeof(T); ++i) { out.at(i) = raw.at(i); }
    return out;
}

template <typename T, int N> static T deserialize(const std::vector<uint8_t>& data) {
    static_assert(std::is_trivially_copyable_v<T>, "Type must be trivially copyable");
    std::array<uint8_t, N> raw;
    for (int i = 0; i < N; ++i) { raw.at(i) = data.at(i); }
    return std::bit_cast<T>(raw);
}

//////////////////////////////////////
// OTOS
/////////////////

OTOS::OTOS(std::shared_ptr<copro::Coprocessor> coprocessor)
    : m_coprocessor(coprocessor) {}

Status OTOS::get_status() {
    union {
            struct {
                    uint8_t warn_tilt_angle : 1;
                    uint8_t warn_optical_tracking : 1;
                    uint8_t reserved : 4;
                    uint8_t optical_fatal : 1;
                    uint8_t imu_fatal : 1;
            };

            uint8_t value;
    } s;

    auto raw = m_coprocessor->write_and_receive("otos/get_status", {}, READ_TIMEOUT);
    if (raw.empty()) {
        return {0, 0, 0, 0, 1};
    } else {
        s.value = raw.at(0);
        return {static_cast<bool>(s.warn_tilt_angle), static_cast<bool>(s.warn_optical_tracking),
                static_cast<bool>(s.optical_fatal), static_cast<bool>(s.imu_fatal), 0};
    }
}

int OTOS::self_test() {
    const auto raw = m_coprocessor->write_and_receive("otos/self_test", {}, READ_TIMEOUT);
    if (raw.empty()) {
        return PROS_ERR;
    } else {
        return static_cast<int>(raw.at(0));
    }
}

int OTOS::reset_tracking() {
    auto raw = m_coprocessor->write_and_receive("otos/reset_tracking", {}, READ_TIMEOUT);
    if (raw.empty()) {
        return PROS_ERR;
    } else {
        return static_cast<int>(raw.at(0));
    }
}

//////////////////////////////////////
// pose
/////////////////

Pose OTOS::get_pose() {
    constexpr Pose ERROR = {std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity(),
                            std::numeric_limits<float>::infinity()};

    // request, receive
    auto tmp = m_coprocessor->write_and_receive("otos/get_pose", {}, READ_TIMEOUT);
    if (tmp.empty()) return ERROR;

    // error checking
    bool err = true;
    for (uint8_t b : tmp) {
        if (b != 1) err = false;
    }
    if (err) return ERROR;

    // parse raw data
    int16_t rawX = (tmp[1] << 8) | tmp[0];
    int16_t rawY = (tmp[3] << 8) | tmp[2];
    int16_t rawH = (tmp[5] << 8) | tmp[4];

    return {rawX * INT16_TO_INCH, rawY * INT16_TO_INCH, rawH * INT16_TO_DEG};
}

int OTOS::set_pose(Pose pose) {
    // cast
    int16_t rawX = (pose.x * INCH_TO_INT16);
    int16_t rawY = (pose.y * INCH_TO_INT16);
    int16_t rawH = (pose.h * DEG_TO_INT16);
    // init vector
    std::vector<uint8_t> out(6, 0);
    // serialize
    out[0] = rawX & 0xFF;
    out[1] = (rawX >> 8) & 0xFF;
    out[2] = rawY & 0xFF;
    out[3] = (rawY >> 8) & 0xFF;
    out[4] = rawH & 0xFF;
    out[5] = (rawH >> 8) & 0xFF;
    // write and get response
    auto raw = m_coprocessor->write_and_receive("otos/set_pose", out, READ_TIMEOUT);
    if (raw.empty()) return PROS_ERR;
    else return static_cast<int>(raw.at(0));
}

//////////////////////////////////////
// offset
/////////////////

int OTOS::set_offset(Pose pose) {
    // cast
    int16_t rawX = (pose.x * INCH_TO_INT16);
    int16_t rawY = (pose.y * INCH_TO_INT16);
    int16_t rawH = (pose.h * DEG_TO_INT16);
    // init vector
    std::vector<uint8_t> out(6, 0);
    // serialize
    out[0] = rawX & 0xFF;
    out[1] = (rawX >> 8) & 0xFF;
    out[2] = rawY & 0xFF;
    out[3] = (rawY >> 8) & 0xFF;
    out[4] = rawH & 0xFF;
    out[5] = (rawH >> 8) & 0xFF;
    // write and get response
    auto raw = m_coprocessor->write_and_receive("otos/set_offset", out, READ_TIMEOUT);
    if (raw.empty()) return PROS_ERR;
    else return static_cast<int>(raw.at(0));
}

//////////////////////////////////////
// linear scalar
/////////////////

float OTOS::get_linear_scalar() {
    auto raw = m_coprocessor->write_and_receive("otos/get_linear_scalar", {}, READ_TIMEOUT);
    if (raw.empty()) return std::numeric_limits<float>::infinity();
    else return 0.001f * static_cast<int8_t>(raw.at(0)) + 1.0f;
}

int OTOS::set_linear_scalar(float scalar) {
    auto raw = static_cast<uint8_t>((scalar - 1.0f) * 1000 + 0.5f);
    auto err = m_coprocessor->write_and_receive("otos/set_linear_scalar", {raw}, READ_TIMEOUT);
    if (err.empty()) return PROS_ERR;
    else return 0;
}

//////////////////////////////////////
// angular scalar
/////////////////

float OTOS::get_angular_scalar() {
    auto raw = m_coprocessor->write_and_receive("otos/get_angular_scalar", {}, READ_TIMEOUT);
    if (raw.empty()) return std::numeric_limits<float>::infinity();
    else return 0.001f * static_cast<int8_t>(raw.at(0)) + 1.0f;
}

int OTOS::set_angular_scalar(float scalar) {
    auto raw = static_cast<uint8_t>((scalar - 1.0f) * 1000 + 0.5f);
    auto err = m_coprocessor->write_and_receive("otos/set_angular_scalar", {raw}, READ_TIMEOUT);
    if (err.empty()) return PROS_ERR;
    else return 0;
}

//////////////////////////////////////
// calibrate
/////////////////

int OTOS::calibrate(uint8_t samples) {
    auto err = m_coprocessor->write_and_receive("otos/calibrate", {samples}, READ_TIMEOUT);
    if (err.empty() || (err.at(0) != 0 && err.at(0) != 1)) return PROS_ERR;
    return err.at(0);
}

int OTOS::is_calibrated() {
    auto err = m_coprocessor->write_and_receive("otos/is_calibrated", {}, READ_TIMEOUT);
    if (err.empty() || (err.at(0) != 0 && err.at(0) != 1)) return PROS_ERR;
    return err.at(0);
}

} // namespace copro