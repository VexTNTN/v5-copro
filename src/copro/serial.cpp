#include "copro/serial.hpp"
#include "copro/codec.hpp"
#include "pros/rtos.hpp"
#include "pros/serial.hpp"
#include <iostream>

namespace copro {

static std::vector<uint8_t> rx_buffer; // Reusable buffer for receiving

std::span<const uint8_t> read(pros::Serial serial) {
  constexpr std::array<uint8_t, 2> START_DELIM = {0xAA, 0x55};
  constexpr std::array<uint8_t, 2> END_DELIM = {0x55, 0xAA};
  constexpr int64_t TIMEOUT_MS = 100;

  auto read_byte = [&]() -> std::optional<uint8_t> {
    int start = pros::millis();
    while (serial.get_read_avail() < 1) {
      if (pros::millis() - start > 1000) {
        std::cout << "timed out" << std::endl;
        return std::nullopt;
      }
    }
    int b = serial.read_byte();
    return b != -1 ? std::make_optional(static_cast<uint8_t>(b)) : std::nullopt;
  };

  // 1. Find start delimiters
  while (true) {
    auto b1 = read_byte();
    if (!b1 || *b1 != START_DELIM[0]) {
      continue;
    }

    auto b2 = read_byte();
    if (b2 && *b2 == START_DELIM[1]) {
      break;
    }
  }

  // 2. Read header
  struct Header {
    uint16_t payload_len;
    uint16_t header_crc;
  } header;

  for (size_t i = 0; i < sizeof(header); ++i) {
    auto b = read_byte();
    if (!b) {
      return {};
    }
    reinterpret_cast<uint8_t *>(&header)[i] = *b;
  }

  // Validate header CRC
  if (crc16({reinterpret_cast<uint8_t *>(&header.payload_len), 2}) !=
      header.header_crc) {
    return {}; // Bad header
  }

  // 3. Read stuffed payload
  rx_buffer.resize(header.payload_len);
  for (size_t i = 0; i < header.payload_len; ++i) {
    auto b = read_byte();
    if (!b) {
      rx_buffer.clear();
      return {};
    }
    rx_buffer[i] = *b;
  }

  // 4. Read payload CRC
  uint16_t received_crc = 0;
  for (size_t i = 0; i < 2; ++i) {
    auto b = read_byte();
    if (!b) {
      rx_buffer.clear();
      return {};
    }
    reinterpret_cast<uint8_t *>(&received_crc)[i] = *b;
  }

  // 5. Validate end delimiters
  auto end1 = read_byte();
  auto end2 = read_byte();
  if (!end1 || !end2 || *end1 != END_DELIM[0] || *end2 != END_DELIM[1]) {
    rx_buffer.clear();
    return {};
  }

  // 6. Unstuff and validate payload
  std::vector<uint8_t> unstuffed = byte_unstuff(rx_buffer);
  if (crc16(unstuffed) != received_crc) {
    return {};
  }

  // Return valid payload
  rx_buffer = std::move(unstuffed);
  return {rx_buffer.data(), rx_buffer.size()};
}
} // namespace copro