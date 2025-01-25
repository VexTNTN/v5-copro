#pragma once

/**
 * @brief create a checksum using the CCITT-FALSE algorithm (polynomial 0x1021)
 *
 * @param data span of bytes to parse
 * @return uint16_t checksum
 */
#include <cstdint>
#include <span>
#include <vector>

namespace copro {

uint16_t crc16(std::span<const uint8_t> data);

std::vector<uint8_t> byte_unstuff(std::span<const uint8_t> data);

std::vector<uint8_t> byte_stuff(std::span<const uint8_t> data);

} // namespace copro