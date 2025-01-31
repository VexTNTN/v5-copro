#pragma once

#include <cstdint>
#include <vector>

namespace copro {
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
} // namespace copro