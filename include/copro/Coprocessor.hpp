#pragma once

#include <cstdint>
#include <vector>

namespace copro {
std::vector<uint8_t> write_and_receive(uint8_t id, const std::vector<uint8_t>& data, int timeout);
/**
 * @brief initialize comms with the coprocessor.
 *
 * This function will block until the coprocessor responds to a ping.
 * However, if driver control starts at any point, this function will
 * exit with an error
 *
 * the function may set the following values of errno:
 * EINVAL: the port is not in a valid range [1-21]
 * EACCESS: another resource is currently trying to access the port
 * EINTR: driver control started while initializing
 * ETIMEDOUT: could not communicate with pi before timeout ended
 *
 * @param port the port the coprocessor is on
 * @param baud the baud rate of the serial port. Usually 921600
 * @param timeout max time that can be taken to initialize, in milliseconds. -1
 * to disable. -1 by default
 */
int init(int port, int baud, int timeout = -1);
} // namespace copro