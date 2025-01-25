#pragma once

#include "pros/serial.hpp"
#include <cstdint>
#include <span>

namespace copro {

std::span<const uint8_t> read();

}