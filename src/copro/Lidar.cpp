#include "Lidar.hpp"
#include "Coprocessor.hpp"
#include "pros/error.h"
#include <limits>

// TODO: add mutex for copro communication?

namespace lidar {
constexpr int ReadTimeout = 20;
constexpr double kInt16ToRad = 0; // placeholder
constexpr double kInt16ToM = 0; // placeholder

enum class MessageId {
    Start = 50,
    Stop = 51,
    Reset = 52,
    GetHealth = 53,
    GetMeasurement = 54,
};

int start(Id id) noexcept {
    // request structure:
    // [1-bit id] [7-bit padding]
    const auto request = (id == Id::Right) ? static_cast<uint8_t>(1u << 7) :
                                             static_cast<uint8_t>(0);
    const auto raw =
      copro::write_and_receive(static_cast<uint8_t>(MessageId::Start),
                               { request },
                               ReadTimeout);

    if (raw.empty()) {
        return PROS_ERR;
    } else {
        return static_cast<int>(raw.at(0));
    }
}

int stop(Id id) noexcept {
    // request structure:
    // [1-bit id] [7-bit padding]
    const auto request = (id == Id::Right) ? static_cast<uint8_t>(1u << 7) :
                                             static_cast<uint8_t>(0);
    const auto raw =
      copro::write_and_receive(static_cast<uint8_t>(MessageId::Stop),
                               { request },
                               ReadTimeout);

    if (raw.empty()) {
        return PROS_ERR;
    } else {
        return static_cast<int>(raw.at(0));
    }
}

int reset(Id id) noexcept {
    // request structure:
    // [1-bit id] [7-bit padding]
    const auto request = (id == Id::Right) ? static_cast<uint8_t>(1u << 7) :
                                             static_cast<uint8_t>(0);
    const auto raw =
      copro::write_and_receive(static_cast<uint8_t>(MessageId::Reset),
                               { request },
                               ReadTimeout);

    if (raw.empty()) {
        return PROS_ERR;
    } else {
        return static_cast<int>(raw.at(0));
    }
}

Health getHealth(Id id) noexcept {
    // request structure:
    // [1-bit id] [7-bit padding]
    const auto request = (id == Id::Right) ? static_cast<uint8_t>(1u << 7) :
                                             static_cast<uint8_t>(0);
    const auto raw =
      copro::write_and_receive(static_cast<uint8_t>(MessageId::GetHealth),
                               { request },
                               ReadTimeout);

    // only possible due to a bug in the coprocessor firmware,
    // but we check anyway
    if (raw.size() < 3) {
        return { .status = Status::Unknown, .error_code = 0 };
    }

    // response structure:
    // [8-bit status] [16-bit error code]
    const Status status = static_cast<Status>(raw.at(0));
    const uint16_t errorCode =
      static_cast<uint16_t>(raw.at(1) << 8 | static_cast<uint16_t>(raw.at(2)));

    return { status, errorCode };
}

Measurement getMeasurement() noexcept {
    // request structure: none
    const auto raw =
      copro::write_and_receive(static_cast<uint8_t>(MessageId::GetMeasurement),
                               {},
                               ReadTimeout);

    // only possible due to a bug in the coprocessor firmware,
    // but we check anyway
    if (raw.size() < 7) {
        return { Id::Left,
                 Quality::Error,
                 std::numeric_limits<double>::infinity(),
                 std::numeric_limits<double>::infinity(),
                 0 };
    }

    // response structure:
    // [15-bit angle] [1-bit id] [15-bit distance] [1-bit quality]
    // [16-bit timestamp]
    // TODO: these calculations don't look right

    const uint16_t raw_angle =
      static_cast<uint16_t>(raw.at(0) | (raw.at(1) << 8)) & 0x7FFF;
    const double angle = kInt16ToRad * static_cast<double>(raw_angle);

    const Id id = ((raw.at(1) & 0x0001u) != 0) ? Id::Right : Id::Left;

    const uint16_t raw_distance =
      static_cast<uint16_t>(raw.at(2) | (raw.at(3) << 8)) & 0x7FFF;

    const uint8_t quality_bit = (raw.at(3) >> 7) & 0x01;
    const uint32_t raw_timestamp = static_cast<uint32_t>(
      raw.at(4) | (raw.at(5) << 8) | (raw.at(6) << 16) | (raw.at(7) << 24));

    return { id,
             static_cast<Quality>(quality_bit),
             kInt16ToM * static_cast<double>(raw_distance),
             kInt16ToRad * static_cast<double>(raw_angle),
             static_cast<uint64_t>(raw_timestamp) };
}
} // namespace lidar
