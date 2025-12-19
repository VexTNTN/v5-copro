#pragma once

#include <cstdint>

namespace lidar {

enum class Id {
    Left = 0, // the left port on the board
    Right = 1, // the right port on the board
};

enum class Quality {
    Good = 1,
    Poor = 0,
    Error = 2,
};

enum class Status {
    Good = 0,
    Warning = 1,
    Error = 2,
    Unknown = 3,
};

struct Health {
    Status status;
    uint16_t error_code;
};

struct Measurement {
    Id id;
    Quality quality;
    double distance; // in mm
    double angle; // in radians, clockwise from the arrow on the module
    uint64_t micros; // estimated time at which the measurement was taken
};

int start(Id id) noexcept;
int stop(Id id) noexcept;
int reset(Id id) noexcept;
Health getHealth(Id id) noexcept;
Measurement getMeasurement() noexcept;
}; // namespace lidar
