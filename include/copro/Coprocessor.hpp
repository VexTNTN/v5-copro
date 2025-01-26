#pragma once

#include "pros/serial.hpp"
namespace copro {

class Coprocessor {
public:
  /**
   * @brief Create a new Coprocessor
   *
   * @param port the port of the coprocessor [1-21]
   * @param baudrate the baudrate of serial comms to the coprocessor
   */
  Coprocessor(int port, int baudrate);
  std::span<const uint8_t> read();
  void write(std::span<const uint8_t> message);

private:
  pros::Serial m_serial;
  std::vector<uint8_t> m_rx_buffer;
};

} // namespace copro