#pragma once

#include "Coprocessor.hpp"
#include <cmath>
#include <cstdint>
#include <expected>
#include <source_location>
#include <string>
#include <vector>

namespace copro {

struct OtosError {
    enum class Type {
        None,
        CoproInternalIO,
        CoproInternalUnknown,
        EmptyResponse,
        CorruptedResponse,
        NoResponse,
        WrongMessageLength,
        TransportError // Added to cover specific serial port issues
    } type;

    std::string what;
    std::vector<std::source_location> where;
};

// Enable printing OtosError using std::cout if needed
std::ostream& operator<<(std::ostream& os, const OtosError& err);

void print_error(OtosError& error,
                 std::source_location loc = std::source_location::current());

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

class Otos {
  public:
    /**
     * @brief Construct a new Otos device handler
     * @param copro Reference to the initialized Coprocessor instance
     */
    explicit Otos(copro::Coprocessor& copro)
        : m_copro(copro) {}

    // Disable copy/move/destroy
    Otos(const Otos&) = delete;
    Otos& operator=(const Otos&) = delete;
    // ~Otos() = delete;

    [[nodiscard]]
    std::expected<Status, OtosError> get_status();

    [[nodiscard]]
    std::expected<bool, OtosError> self_test();

    [[nodiscard]]
    std::expected<void, OtosError> reset_tracking();

    // --- Pose ---
    [[nodiscard]]
    std::expected<Pose, OtosError> get_pose();

    [[nodiscard]]
    std::expected<void, OtosError> set_pose(Pose pose);

    // --- Acceleration ---
    [[nodiscard]]
    std::expected<Acceleration, OtosError> get_acceleration();

    // --- Linear Scalar ---
    [[nodiscard]]
    std::expected<float, OtosError> get_linear_scalar();

    [[nodiscard]]
    std::expected<void, OtosError> set_linear_scalar(float scalar);

    // --- Angular Scalar ---
    [[nodiscard]]
    std::expected<float, OtosError> get_angular_scalar();

    [[nodiscard]]
    std::expected<void, OtosError> set_angular_scalar(float scalar);

    // --- Calibration ---
    [[nodiscard]]
    std::expected<void, OtosError> calibrate(uint8_t samples);

    [[nodiscard]]
    std::expected<bool, OtosError> is_calibrated();

  private:
    copro::Coprocessor& m_copro;

    // --- Internal Helpers ---
    enum class CoproStatus : std::uint8_t {
        Ok = 0,
        IoError = 2,
    };

    static std::expected<void, OtosError>
    validate_message(const std::vector<uint8_t>& message, size_t expected_len);
};

} // namespace copro
