#pragma once

#include <cstdint>
#include <expected>
#include <source_location>
#include <string>
#include <vector>

namespace otos {

struct OtosError {
    enum class Type {
        None,
        CoproInternalIO,
        CoproInternalUnknown,
        EmptyResponse,
        CorruptedResponse,
        NoResponse,
        WrongMessageLength
    } type;

    std::string what;
    std::vector<std::source_location> where;
};

struct Pose {
    float x;
    float y;
    float h;
};

using Acceleration = Pose;

struct Status {
    bool warn_tilt_angle;
    bool warn_optical_tracking;
    bool optical_fatal;
    bool imu_fatal;
};

std::expected<Status, OtosError> get_status() noexcept;

std::expected<bool, OtosError> self_test() noexcept;

std::expected<void, OtosError> reset_tracking() noexcept;

//////////////////////////////////////
// pose
/////////////////

std::expected<Pose, OtosError> get_pose() noexcept;

std::expected<void, OtosError> set_pose(Pose pose) noexcept;

//////////////////////////////////////
// acceleration
/////////////////

std::expected<Acceleration, OtosError> get_acceleration() noexcept;

//////////////////////////////////////
// offset
/////////////////

std::expected<void, OtosError> set_offset(Pose pose) noexcept;

//////////////////////////////////////
// linear scalar
/////////////////

std::expected<float, OtosError> get_linear_scalar() noexcept;

std::expected<void, OtosError> set_linear_scalar(float scalar) noexcept;

//////////////////////////////////////
// angular scalar
/////////////////

std::expected<float, OtosError> get_angular_scalar() noexcept;

std::expected<void, OtosError> set_angular_scalar(float scalar) noexcept;

//////////////////////////////////////
// calibrate
/////////////////

std::expected<void, OtosError> calibrate(uint8_t samples) noexcept;

std::expected<bool, OtosError> is_calibrated() noexcept;
} // namespace otos
