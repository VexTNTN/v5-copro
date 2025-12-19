#pragma once

#include <cstdint>
#include <vector>

namespace copro {

/**
 * @brief Serialize a trivially copyable type into a vector of bytes
 *
 * @tparam T the datatype
 *
 * @param data the data to serialize
 *
 * @return std::vector<uint8_t> the serialized data
 */
template<typename T>
static std::vector<uint8_t> serialize(const T& data) {
    static_assert(std::is_trivially_copyable_v<T>,
                  "Type must be trivially copyable");
    auto raw = std::bit_cast<std::array<uint8_t, sizeof(T)>>(data);
    std::vector<uint8_t> out(sizeof(T));
    for (int i = 0; i < sizeof(T); ++i) {
        out.at(i) = raw.at(i);
    }
    return out;
}

/**
 * @brief Deserialize a vector of bytes into a type
 *
 * @tparam T the datatype
 * @tparam N the size of the datatype in bytes
 *
 * @param data the data to deserialize
 *
 * @return T the deserialized data
 */
template<typename T, int N>
static T deserialize(const std::vector<uint8_t>& data) {
    static_assert(std::is_trivially_copyable_v<T>,
                  "Type must be trivially copyable");
    std::array<uint8_t, N> raw;
    for (int i = 0; i < N; ++i) {
        raw.at(i) = data.at(i);
    }
    return std::bit_cast<T>(raw);
}

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
void write(const std::vector<uint8_t>& message);

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

/**
 * @brief Write a message to the coprocessor and wait for a response
 *
 * @param id the message ID
 * @param data the payload data
 * @param timeout how long to wait for a response, in milliseconds
 *
 * @return std::vector<uint8_t> the response payload
 */
std::vector<uint8_t> write_and_receive(uint8_t id,
                                       const std::vector<uint8_t>& data,
                                       int timeout) noexcept;
} // namespace copro
