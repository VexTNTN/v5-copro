#include "copro/Coprocessor.hpp"
#include "pros/error.h"
#include "pros/serial.h"
#include <expected>
#include <format>
#include <limits>
#include <mutex>

namespace copro {

//////////////////////////////////////
// using
/////////////////

using Err = Error<Coprocessor::ErrorType>;
using enum Coprocessor::ErrorType;

//////////////////////////////////////
// util functions
/////////////////

/**
 * @brief calculate a CRC16 using CCITT-FALSE
 *
 * @param data the data to generate a crc16 for
 * @return uint16_t the crc16
 */
static uint16_t crc16(const std::vector<uint8_t>& data) {
    // lookup table for extra zoom
    // (copy/paste was easier than implementing the algorithm)
    constexpr std::array<uint16_t, 256> table = {
        0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7, 0x8108,
        0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF, 0x1231, 0x0210,
        0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6, 0x9339, 0x8318, 0xB37B,
        0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE, 0x2462, 0x3443, 0x0420, 0x1401,
        0x64E6, 0x74C7, 0x44A4, 0x5485, 0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE,
        0xF5CF, 0xC5AC, 0xD58D, 0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6,
        0x5695, 0x46B4, 0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D,
        0xC7BC, 0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
        0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B, 0x5AF5,
        0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12, 0xDBFD, 0xCBDC,
        0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A, 0x6CA6, 0x7C87, 0x4CE4,
        0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41, 0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD,
        0xAD2A, 0xBD0B, 0x8D68, 0x9D49, 0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13,
        0x2E32, 0x1E51, 0x0E70, 0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A,
        0x9F59, 0x8F78, 0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E,
        0xE16F, 0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
        0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E, 0x02B1,
        0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256, 0xB5EA, 0xA5CB,
        0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D, 0x34E2, 0x24C3, 0x14A0,
        0x0481, 0x7466, 0x6447, 0x5424, 0x4405, 0xA7DB, 0xB7FA, 0x8799, 0x97B8,
        0xE75F, 0xF77E, 0xC71D, 0xD73C, 0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657,
        0x7676, 0x4615, 0x5634, 0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9,
        0xB98A, 0xA9AB, 0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882,
        0x28A3, 0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
        0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92, 0xFD2E,
        0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9, 0x7C26, 0x6C07,
        0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1, 0xEF1F, 0xFF3E, 0xCF5D,
        0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8, 0x6E17, 0x7E36, 0x4E55, 0x5E74,
        0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
    };

    uint16_t crc = 0xFFFF;
    for (uint8_t byte : data) {
        crc = (crc << 8) ^ table[((crc >> 8) ^ byte) & 0xFF];
    }
    return crc;
}

/**
 * @brief append a vector to the another vector
 *
 * @tparam T the type of vector
 *
 * @param v the vector to append to
 * @param data the vector that will be appended
 *
 * @return std::vector<T> the vector that was appended to
 */
template<typename T>
static std::vector<T> vector_append(std::vector<T>& v,
                                    const std::vector<T>& data) {
    for (const auto b : data) v.push_back(b);
    return v;
}

/**
 * @brief get an object as a vector of uint8_t
 *
 * @tparam T the type of the object
 *
 * @param t the object to serialize
 *
 * @return std::vector<uint8_t> the object as a vector of uint8_t
 */
template<typename T>
static std::vector<uint8_t> serialize(T t) {
    const auto raw = std::bit_cast<std::array<uint8_t, sizeof(T)>>(t);
    std::vector<uint8_t> out;
    for (const uint8_t b : raw) out.push_back(b);
    return out;
}

//////////////////////////////////////
// i/o helpers
/////////////////

/**
 * @brief enable a port as generic serial port
 *
 * @param port the port to initialize as a generic serial port
 *
 * @return void on success
 * @return Err on failure
 */
static std::expected<void, Err> enable_serial(int port) {
    if (pros::c::serial_enable(port) == PROS_ERR) {
        if (errno == EINVAL)
            return Err::make(INVALID_PORT, "port {} is not valid", port);
        if (errno == EACCES)
            return Err::make(PORT_UNAVAILABLE,
                             "port {} is being accessed by another resource",
                             port);
        return Err::make(UNKNOWN_FAILURE, "unknown failure on port {}", port);
    }
    // vexos needs some time to enable serial mode on the port
    pros::delay(15);
    return {};
}

/**
 * @brief set the baud rate of a generic serial port
 *
 * @param port the port to set the baud rate on
 *
 * @return void on success
 * @return Err on failure
 */
static std::expected<void, Err> set_baud_rate(int port, int baud_rate) {
    if (pros::c::serial_set_baudrate(port, baud_rate) == PROS_ERR) {
        if (errno == EINVAL)
            return Err::make(INVALID_PORT, "port {} is not valid", port);
        if (errno == EACCES)
            return Err::make(PORT_UNAVAILABLE,
                             "port {} is being accessed by another resource",
                             port);
        return Err::make(UNKNOWN_FAILURE, "unknown failure on port {}", port);
    }
    // vexos needs some time to set the baud rate
    pros::delay(15);
    return {};
}

/**
 * @brief flush the input and output buffers of a generic serial port
 *
 * @param port the port to flush the buffers of
 *
 * @return void on success
 * @return Err on failure
 */
static std::expected<void, Err> flush(int port) {
    if (pros::c::serial_flush(port) == PROS_ERR) {
        if (errno == EINVAL)
            return Err::make(INVALID_PORT, "port {} is not valid", port);
        if (errno == EACCES)
            return Err::make(PORT_UNAVAILABLE,
                             "port {} is being accessed by another resource",
                             port);
        return Err::make(UNKNOWN_FAILURE, "unknown failure on port {}", port);
    }
    // vexos needs some time to flush the serial port
    pros::delay(15);
    return {};
}

/**
 * @brief read a single byte from a generic serial port
 *
 * @param port the generic serial port to read from
 * @return uint8_t the read byte, on success
 * @return Err on failure
 */
static std::expected<uint8_t, Err> read_byte(int port) {
    int32_t raw = pros::c::serial_read_byte(port);
    // Handle timeout scenario with single retry
    if (raw == -1) {
        pros::delay(1);
        raw = pros::c::serial_read_byte(port);
        if (raw == -1)
            return Err::make(READ_TIMEOUT,
                             "transmission stopped abruptly on port {}",
                             port);
    }
    // Handle PROS error
    if (raw == PROS_ERR) {
        if (errno == EINVAL)
            return Err::make(INVALID_PORT, "port {} is not valid", port);
        if (errno == EACCES)
            return Err::make(PORT_UNAVAILABLE,
                             "port {} is being accessed by another resource",
                             port);
        return Err::make(UNKNOWN_FAILURE,
                         "unknown failure on generic serial port {}",
                         port);
    }
    return static_cast<uint8_t>(raw);
}

/**
 * @brief read an object from the stream
 *
 * @tparam T the type of the object
 *
 * @param port the generic serial port to read from
 *
 * @return T the object, on success
 * @return Err on failure
 */
template<typename T>
static std::expected<T, Err> read_stream(int port) {
    std::array<uint8_t, sizeof(T)> raw;
    for (uint8_t& b : raw) {
        if (auto c = read_byte(port)) {
            b = c.value();
        } else {
            return Err::add(c, "failed to read from serial stream");
        }
    }
    return std::bit_cast<T>(raw);
}

/**
 * @brief read all available bytes on a generic serial port
 *
 * @param port the generic serial port to read from
 *
 * @return std::vector<uint8_t> the read bytes, on success
 * @return Err on failure
 */
static std::expected<std::vector<uint8_t>, Err> read(int port) {
    // check if there's data available
    const int avail = pros::c::serial_get_read_avail(port);
    if (avail == 0) return Err::make(NO_MESSAGE, "no message available");
    if (avail == PROS_ERR) {
        if (errno == EINVAL) {
            return Err::make(INVALID_PORT, "port {} is not valid", port);
        } else if (errno == EACCES) {
            return Err::make(PORT_UNAVAILABLE,
                             "port {} is being accessed by another resource",
                             port);
        } else {
            return Err::make(UNKNOWN_FAILURE,
                             "unknown failure on generic serial port {}",
                             port);
        }
    }

    // read the header
    const auto crc = read_stream<uint16_t>(port);
    if (!crc) return Err::add(crc, "failed to read CRC");

    // read the data
    std::vector<uint8_t> payload;
    while (true) {
        const auto byte = read_byte(port);
        if (!byte) return Err::add(byte, "failed to read payload");
        payload.push_back(*byte);
    }

    // validate payload with CRC
    if (crc != crc16(payload)) {
        return Err::make(MESSAGE_CORRUPTED,
                         "message corrupted on port {}, CRC failed",
                         port);
    }

    // return payload
    return payload;
}

/**
 * @brief write a vector of bytes to a generic serial port
 *
 * @param port the generic serial port to write to
 *
 * @return void on success
 * @return Err on failure
 */
static std::expected<void, Err> write(const std::vector<uint8_t>& message,
                                      int port) {
    std::vector<uint8_t> out;
    // add the CRC16
    vector_append(out, serialize(crc16(message)));
    // add the message
    vector_append(out, message);

    // write encoded message
    if (pros::c::serial_write(port, out.data(), out.size()) == PROS_ERR) {
        if (errno == EINVAL)
            return Err::make(INVALID_PORT, "port {} is not valid", port);
        if (errno == EACCES)
            return Err::make(PORT_UNAVAILABLE,
                             "port {} is being accessed by another resource",
                             port);
        if (errno == EIO)
            return Err::make(IO_FAILURE, "system IO failure on port {}", port);
        return Err::make(UNKNOWN_FAILURE,
                         "unknown failure on generic serial port {}",
                         port);
    }
    return {};
}

//////////////////////////////////////
// member functions
/////////////////

Coprocessor::Coprocessor(int port, int baud_rate)
    : m_port(port),
      m_baud_rate(baud_rate) {}

std::expected<std::vector<uint8_t>, Err>
Coprocessor::write_and_receive(const std::string& topic,
                               const std::vector<uint8_t>& data,
                               int timeout) {
    // if not initialized, bail
    if (!m_initialized)
        return Err::make(NOT_INITIALIZED,
                         "port {} not initialized as a generic serial device",
                         m_port);
    // mutex
    std::lock_guard lock(m_mutex);
    // prepare data
    std::vector<uint8_t> out;
    auto id = find_id(topic);
    if (!id) return std::unexpected(id.error());
    out.push_back(*id);
    out.insert(out.end(), data.begin(), data.end());
    // write data
    {
        auto err = write(out, m_port);
        if (!err) return std::unexpected(err.error());
    }
    // wait for a response
    const int start = pros::millis();
    while (true) {
        auto raw = read(m_port);
        if (raw) {
            std::vector<uint8_t> rtn;
            rtn.insert(rtn.end(), raw->begin() + 1, raw->end());
            return rtn;
        }
        // handle the error
        if (raw.error().type == READ_TIMEOUT &&
            pros::millis() - start > timeout)
            return std::unexpected(raw.error());
        else if (raw.error().type != READ_TIMEOUT)
            return std::unexpected(raw.error());
    }
}

std::expected<int, Err> Coprocessor::initialize() {
    if (m_initialized) { // check if already initialized
        return Err::make(
          ALREADY_INITIALIZED,
          "port {} has already been initialized as a generic serial device",
          m_port);
    }
    { // enable serial port
        auto err = enable_serial(m_port);
        if (!err) return Err::add(err, "failed to enable serial port");
    }
    { // set baud rate
        auto err = set_baud_rate(m_port, m_baud_rate);
        if (!err) return Err::add(err, "failed to set baud rate");
    }
    { // flush serial buffer
        auto err = flush(m_port);
        if (!err) return Err::add(err, "failed to flush serial port");
    }
    // wait for the coprocessor to boot
    while (true) {
        auto err = write_and_receive("ping", {}, 10);
        if (err) {
            m_initialized = true;
            return 0;
        } else if (err.error().type != READ_TIMEOUT) {
            return Err::add(err, "failed parsing ping response");
        }
        pros::delay(10);
    }
}

int Coprocessor::get_port() {
    return m_port;
}

std::expected<uint8_t, Err> Coprocessor::find_id(const std::string& topic) {
    // if not initialized, bail
    if (!m_initialized)
        return Err::make(NOT_INITIALIZED,
                         "port {} not initialized as a generic serial device",
                         m_port);
    // if the topic is already registered, return its index
    for (int i = 0; i < m_topics.size(); i++) {
        if (m_topics.at(i) == topic) return i;
    }
    // check that there's space for a new topic
    if (m_topics.size() >= std::numeric_limits<uint8_t>::max()) {
        return Err::make(TOO_MANY_TOPICS,
                         "too many topics registered on coprocessor on port {}",
                         m_port);
    }
    // assemble message
    std::vector<uint8_t> out;
    out.push_back(m_topics.size());
    for (uint8_t c : topic) out.push_back(c);
    auto err = write_and_receive("register", out, 5);
    // check for errors
    if (!err) return Err::add(err, "failed to register topic");
    if (err->at(0) != 0) {
        return Err::make(UNKNOWN_FAILURE_COPROCESSOR,
                         "unknown failure from coprocessor on port {}",
                         m_port);
    }
    // add the topic to the vector
    m_topics.push_back(topic);
    return m_topics.size() - 1;
}
} // namespace copro
