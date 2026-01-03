#include "OTOS.hpp"
#include <algorithm>
#include <bit>
#include <cmath>
#include <format>
#include <iostream>
#include <limits>

using copro::CoproError;

namespace copro {

// --- OtosError Implementation ---

std::ostream& operator<<(std::ostream& os, const OtosError& err) {
    os << "[OtosError] Type: " << (int)err.type << " | Msg: \"" << err.what
       << "\"\n";
    if (!err.where.empty()) {
        os << "  Trace:\n";
        for (const auto& loc : err.where) {
            os << "    at " << loc.function_name() << " (" << loc.file_name()
               << ":" << loc.line() << ")\n";
        }
    }
    return os;
}

namespace { // Internal Linkage

constexpr int READ_TIMEOUT = 30;

// --- Constants ---
constexpr float RAD_TO_DEG = 180.0 / 3.14159;
constexpr float DEG_TO_RAD = 3.14159 / 180.0;
constexpr float METER_TO_INCH = 39.3701;
constexpr float INCH_TO_METER = 1.0 / METER_TO_INCH;

constexpr float METER_TO_INT16 = 32768.0 / 10.0;
constexpr float INT16_TO_METER = 1.0 / METER_TO_INT16;
constexpr float INT16_TO_INCH = INT16_TO_METER * METER_TO_INCH;
constexpr float INCH_TO_INT16 = 1.0 / INT16_TO_INCH;

constexpr float RAD_TO_INT16 = 32768.0 / 3.14159;
constexpr float INT16_TO_RAD = 1.0 / RAD_TO_INT16;
constexpr float INT16_TO_DEG = INT16_TO_RAD * RAD_TO_DEG;
constexpr float DEG_TO_INT16 = 1.0 / INT16_TO_DEG;

static constexpr float MPSS_TO_INT16 = 32768.0f / (16.0f * 9.80665f);
static constexpr float INT16_TO_MPSS = 1.0f / MPSS_TO_INT16;

static constexpr float RPSS_TO_INT16 = 32768.0f / (M_PI * 1000.0f);
static constexpr float INT16_TO_RPSS = 1.0f / RPSS_TO_INT16;

// --- Helper Functions ---

static constexpr std::int16_t to_i16(float v) noexcept {
    constexpr auto I16_MIN = std::numeric_limits<std::int16_t>::min();
    constexpr auto I16_MAX = std::numeric_limits<std::int16_t>::max();

    if (std::isnan(v)) return I16_MAX;
    if (std::isinf(v)) return std::signbit(v) ? I16_MIN : I16_MAX;

    double d = std::clamp(static_cast<double>(v),
                          static_cast<double>(I16_MIN),
                          static_cast<double>(I16_MAX));
    long long r = static_cast<long long>(std::round(d));
    return static_cast<std::int16_t>(r);
}

static constexpr std::int8_t to_i8(float v) noexcept {
    constexpr std::int8_t I8_MIN = -128;
    constexpr std::int8_t I8_MAX = 127;

    if (std::isnan(v)) return I8_MAX;
    if (std::isinf(v)) return std::signbit(v) ? I8_MIN : I8_MAX;

    double d =
      std::clamp(static_cast<double>(v), double(I8_MIN), double(I8_MAX));
    long long r = static_cast<long long>(std::round(d));
    return static_cast<std::int8_t>(r);
}

// --- Error Conversion Helpers ---

OtosError make_otos_error(OtosError::Type type, std::string what) {
    return OtosError { type,
                       std::move(what),
                       { std::source_location::current() } };
}

// Convert low-level CoproError to high-level OtosError
OtosError from_copro(copro::CoproError err) {
    OtosError::Type type;
    switch (err.type) {
        case CoproError::Type::TimedOut:
            type = OtosError::Type::NoResponse;
            break;
        case CoproError::Type::NoData:
            type = OtosError::Type::NoResponse;
            break;
        case CoproError::Type::CorruptedRead:
            type = OtosError::Type::CorruptedResponse;
            break;
        case CoproError::Type::DataCutOff:
            type = OtosError::Type::CorruptedResponse;
            break;
        case CoproError::Type::BrainIoError:
            type = OtosError::Type::CoproInternalIO;
            break;
        case CoproError::Type::MessageTooBig:
            type = OtosError::Type::WrongMessageLength;
            break;
        default: type = OtosError::Type::TransportError; break;
    }

    OtosError out;
    out.type = type;
    out.what = std::move(err.what);
    out.where = std::move(err.where);
    out.where.push_back(std::source_location::current());
    return out;
}

// --- Error Mapping Overloads ---

// Case 1: Convert CoproError -> OtosError
OtosError map_failure(copro::CoproError err, std::source_location loc) {
    auto out = from_copro(std::move(err));
    out.where.push_back(loc); // Add the location where TRY was called
    return out;
}

// Case 2: Pass-through OtosError (already converted or created)
OtosError map_failure(OtosError err, std::source_location loc) {
    err.where.push_back(loc); // Add the location where TRY was called
    return err;
}

// Helper to conditionally move the value only if it isn't void
template<typename T, typename E>
constexpr auto unwrap(std::expected<T, E>& expected) {
    if constexpr (std::is_void_v<T>) {
        return;
    } else {
        return std::move(*expected);
    }
}

} // anonymous namespace

// Defines for the TRY macro to work within member functions.
// Automatically converts CoproError to OtosError if encountered.
#define TRY(expr)                                                        \
    ({                                                                   \
        auto _res = (expr);                                              \
        if (!_res.has_value()) {                                         \
            /* Automatically selects the correct map_failure overload */ \
            return std::unexpected(                                      \
              map_failure(std::move(_res.error()),                       \
                          std::source_location::current()));             \
        }                                                                \
        unwrap(_res);                                                    \
    })

// --- Class Implementation ---

std::expected<void, OtosError>
Otos::validate_message(const std::vector<uint8_t>& message,
                       size_t expected_len) {
    if (message.empty()) {
        return std::unexpected(
          make_otos_error(OtosError::Type::EmptyResponse,
                          "Empty response from coprocessor"));
    }

    // Check status byte (0 = Ok, 2 = IoError, etc.)
    auto status = static_cast<CoproStatus>(message.at(0));
    if (status != CoproStatus::Ok) {
        if (status == CoproStatus::IoError) {
            return std::unexpected(
              make_otos_error(OtosError::Type::CoproInternalIO,
                              "IO error reported by coprocessor"));
        }
        return std::unexpected(
          make_otos_error(OtosError::Type::CoproInternalUnknown,
                          "Unknown error reported by coprocessor"));
    }

    if (message.size() != expected_len) {
        return std::unexpected(make_otos_error(
          OtosError::Type::WrongMessageLength,
          std::format("Invalid response length! Expected {} but has {}",
                      expected_len,
                      message.size())));
    }

    return {};
}

std::expected<Status, OtosError> Otos::get_status() noexcept {
    auto raw = TRY(
      m_copro.write_and_receive(MessageId::GetOtosStatus, {}, READ_TIMEOUT));

    TRY(validate_message(raw, 2));

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

    s.value = raw.at(1);

    return Status {
        static_cast<bool>(s.warn_tilt_angle),
        static_cast<bool>(s.warn_optical_tracking),
        static_cast<bool>(s.optical_fatal),
        static_cast<bool>(s.imu_fatal),
    };
}

std::expected<bool, OtosError> Otos::self_test() noexcept {
    auto raw =
      TRY(m_copro.write_and_receive(MessageId::OtosSelfTest, {}, READ_TIMEOUT));

    TRY(validate_message(raw, 1));

    return true;
}

std::expected<void, OtosError> Otos::reset_tracking() noexcept {
    auto raw = TRY(m_copro.write_and_receive(MessageId::OtosResetTracking,
                                             {},
                                             READ_TIMEOUT));

    TRY(validate_message(raw, 1));

    return {};
}

std::expected<Pose, OtosError> Otos::get_pose() noexcept {
    auto raw =
      TRY(m_copro.write_and_receive(MessageId::OtosGetPose, {}, READ_TIMEOUT));

    TRY(validate_message(raw, 7));

    int16_t rawX = (raw.at(2) << 8) | raw.at(1);
    int16_t rawY = (raw.at(4) << 8) | raw.at(3);
    int16_t rawH = (raw.at(6) << 8) | raw.at(5);

    return Pose { rawX * INT16_TO_INCH,
                  rawY * INT16_TO_INCH,
                  rawH * INT16_TO_DEG };
}

std::expected<void, OtosError> Otos::set_pose(Pose pose) noexcept {
    int16_t rawX = to_i16(pose.x * INCH_TO_INT16);
    int16_t rawY = to_i16(pose.y * INCH_TO_INT16);
    int16_t rawH = to_i16(pose.h * DEG_TO_INT16);

    std::vector<uint8_t> out(6);
    out.at(0) = rawX & 0xFF;
    out.at(1) = (rawX >> 8) & 0xFF;
    out.at(2) = rawY & 0xFF;
    out.at(3) = (rawY >> 8) & 0xFF;
    out.at(4) = rawH & 0xFF;
    out.at(5) = (rawH >> 8) & 0xFF;

    auto raw =
      TRY(m_copro.write_and_receive(MessageId::OtosSetPose, out, READ_TIMEOUT));

    TRY(validate_message(raw, 1));

    // Verify
    auto p = TRY(get_pose());

    if (std::abs(p.x - pose.x) > 1.0f || std::abs(p.y - pose.y) > 1.0f ||
        std::abs(p.h - pose.h) > 1.0f) {
        return std::unexpected(make_otos_error(OtosError::Type::CoproInternalIO,
                                               "SetPose verification failed"));
    }

    return {};
}

std::expected<Acceleration, OtosError> Otos::get_acceleration() noexcept {
    auto raw =
      TRY(m_copro.write_and_receive(MessageId::OtosGetAccel, {}, READ_TIMEOUT));

    TRY(validate_message(raw, 7));

    int16_t rawX = (raw.at(2) << 8) | raw.at(1);
    int16_t rawY = (raw.at(4) << 8) | raw.at(3);
    int16_t rawH = (raw.at(6) << 8) | raw.at(5);

    return Acceleration { rawX * INT16_TO_MPSS * METER_TO_INCH,
                          rawY * INT16_TO_MPSS * METER_TO_INCH,
                          static_cast<float>(rawH * INT16_TO_RPSS / 360.0) };
}

std::expected<void, OtosError> Otos::set_offset(Pose pose) noexcept {
    int16_t rawX = to_i16(pose.x * INCH_TO_INT16);
    int16_t rawY = to_i16(pose.y * INCH_TO_INT16);
    int16_t rawH = to_i16(pose.h * DEG_TO_INT16);

    std::vector<uint8_t> out(6, 0);
    out.at(0) = rawX & 0xFF;
    out.at(1) = (rawX >> 8) & 0xFF;
    out.at(2) = rawY & 0xFF;
    out.at(3) = (rawY >> 8) & 0xFF;
    out.at(4) = rawH & 0xFF;
    out.at(5) = (rawH >> 8) & 0xFF;

    auto raw = TRY(
      m_copro.write_and_receive(MessageId::OtosSetOffset, out, READ_TIMEOUT));

    TRY(validate_message(raw, 1));

    return {};
}

std::expected<float, OtosError> Otos::get_linear_scalar() noexcept {
    auto raw = TRY(m_copro.write_and_receive(MessageId::OtosGetLinearScalar,
                                             {},
                                             READ_TIMEOUT));

    TRY(validate_message(raw, 2));

    return 0.001f * static_cast<int8_t>(raw.at(1)) + 1.0f;
}

std::expected<void, OtosError> Otos::set_linear_scalar(float scalar) noexcept {
    const float scaled = (scalar - 1.0f) * 1000.0f;
    const std::int8_t raw_i8 = to_i8(scaled);
    const std::uint8_t raw_u8 = std::bit_cast<std::uint8_t>(raw_i8);

    auto raw = TRY(m_copro.write_and_receive(MessageId::OtosSetLinearScalar,
                                             { raw_u8 },
                                             READ_TIMEOUT));

    TRY(validate_message(raw, 1));

    // Verify
    auto s = TRY(get_linear_scalar());
    if (std::abs(s - scalar) > 0.02f) {
        return std::unexpected(
          make_otos_error(OtosError::Type::CoproInternalIO,
                          "SetLinearScalar verification failed"));
    }

    return {};
}

std::expected<float, OtosError> Otos::get_angular_scalar() noexcept {
    auto raw = TRY(m_copro.write_and_receive(MessageId::OtosGetAngularScalar,
                                             {},
                                             READ_TIMEOUT));

    TRY(validate_message(raw, 2));

    return 0.001f * static_cast<int8_t>(raw.at(1)) + 1.0f;
}

std::expected<void, OtosError> Otos::set_angular_scalar(float scalar) noexcept {
    const float scaled = (scalar - 1.0f) * 1000.0f;
    const std::int8_t raw_i8 = to_i8(scaled);
    const std::uint8_t raw_u8 = std::bit_cast<std::uint8_t>(raw_i8);

    auto raw = TRY(m_copro.write_and_receive(MessageId::OtosSetAngularScalar,
                                             { raw_u8 },
                                             READ_TIMEOUT));

    TRY(validate_message(raw, 1));

    // Verify
    auto s = TRY(get_angular_scalar());
    if (std::abs(s - scalar) > 0.02f) {
        return std::unexpected(
          make_otos_error(OtosError::Type::CoproInternalIO,
                          "SetAngularScalar verification failed"));
    }

    return {};
}

std::expected<void, OtosError> Otos::calibrate(uint8_t samples) noexcept {
    auto raw = TRY(
      m_copro.write_and_receive(MessageId::OtosCalibrate, { samples }, 1500));

    TRY(validate_message(raw, 1));

    return {};
}

std::expected<bool, OtosError> Otos::is_calibrated() noexcept {
    auto raw = TRY(
      m_copro.write_and_receive(MessageId::OtosIsCalibrated, {}, READ_TIMEOUT));

    TRY(validate_message(raw, 2));
    return static_cast<bool>(raw.at(1));
}

} // namespace copro
