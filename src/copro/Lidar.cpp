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

int start(Id id) noexcept {}

int stop(Id id) noexcept {}

int reset(Id id) noexcept {}

Health getHealth(Id id) noexcept {}

Measurement getMeasurement() noexcept {}
} // namespace lidar
