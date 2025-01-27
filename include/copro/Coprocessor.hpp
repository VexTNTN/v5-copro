#pragma once

#include "pros/serial.hpp"
namespace copro {

class Coprocessor {
public:
  /**
   * @brief Create a new Coprocessor
   *
   * @param port the port of the coprocessor [1-21]
   */
  Coprocessor(int port, int baud);
  /**
   * @brief Read a packet, parse it, and return its payload
   *
   * This function may throw an std::system_error exception,
   * with one of the following error codes:
   *
   * EINVAL - The given value is not within the range of V5 ports (1-21).
   * EACCES - Another resource is currently trying to access the port.
   * EIO - Transmission stopped abruptly.
   * ENODATA - There's no data to read.
   * EPROTO - corrupted escape character, or dropped byte.
   * EBADMSG - data corrupted, CRC check failed
   *
   * @return std::span<const uint8_t> the payload
   */
  std::vector<uint8_t> read(uint32_t timeout);
  void write(std::vector<uint8_t> message);

private:
  uint8_t peek_byte();
  uint8_t read_byte();
  template <typename T> T read_stream();
  pros::Serial m_serial;
  const long int m_timeout;
};

} // namespace copro