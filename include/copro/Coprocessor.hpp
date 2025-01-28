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
   * @brief Write a vector of bytes of the serial port
   *
   * This function may throw an std::system_error exception,
   * with one of the following error codes
   *
   * EOVERFLOW - the message is too large to send
   * EINVAL - The given value is not within the range of V5 ports (1-21).
   * EACCES - Another resource is currently trying to access the port.
   * EIO - "Serious" internal write error.
   *
   * @param message
   */
  void write(const std::vector<uint8_t> &message);
  /**
   * @brief Read a packet, parse it, and return the message
   *
   * This function may throw an std::system_error exception,
   * with one of the following error codes:
   *
   * EINVAL - The given value is not within the range of V5 ports (1-21).
   * EACCES - Another resource is currently trying to access the port.
   * ENODATA - There's no data to read.
   * EBADMSG - data corrupted, CRC check failed
   * EPROTO - corrupted escape character, or dropped byte.
   * ENOLINK - Transmission stopped abruptly.
   *
   * @return std::span<const uint8_t> the payload
   */
  std::vector<uint8_t> read();

private:
  /**
   * @brief read the next byte in the serial stream without removing it
   *
   * This function may throw an std::system_error exception,
   * with one of the following error codes:
   *
   * EINVAL - The given value is not within the range of V5 ports (1-21).
   * EACCES - Another resource is currently trying to access the port.
   * ENOLINK - Transmission stopped abruptly.
   *
   * @return uint8_t
   */
  uint8_t peek_byte();
  /**
   * @brief read the next byte in the serial stream
   *
   * This function may throw an std::system_error exception,
   * with one of the following error codes:
   *
   * EINVAL - The given value is not within the range of V5 ports (1-21).
   * EACCES - Another resource is currently trying to access the port.
   * ENOLINK - Transmission stopped abruptly.
   *
   * @return uint8_t
   */
  uint8_t read_byte();
  /**
   * @brief read the serial stream, and deserialize a type using the next bytes
   *
   * @tparam T the datatype to serialize (has to be trivially-copyable)
   * @return T
   */
  template <typename T> T read_stream();
  pros::Serial m_serial;
  const long int m_timeout;
};

} // namespace copro