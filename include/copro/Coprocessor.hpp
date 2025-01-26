#pragma once

#include "pros/serial.hpp"
namespace copro {

class Coprocessor {
public:
  /**
   * @brief Create a new Coprocessor
   *
   * The serial port will always be set to 921600 baud.
   * Why? Because i didn't have the time to implement it
   *
   * @param port the port of the coprocessor [1-21]
   */
  Coprocessor(int port);
  /**
   * @brief Read a packet, parse it, and return its payload
   *
   * This function may throw an std::system_error exception,
   * with one of the following error codes:
   *
   * ENODATA - there's no data available in the stream
   * ETIMEDOUT - the packet transmission stopped
   *
   * @return std::span<const uint8_t> the payload
   */
  std::span<const uint8_t> read();
  void write(std::span<const uint8_t> message);

private:
  pros::Serial m_serial;
  std::vector<uint8_t> m_rx_buffer;
};

} // namespace copro