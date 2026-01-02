#include "Coprocessor.hpp"
#include "pros/error.h"
#include "pros/rtos.hpp"
#include "pros/serial.h"
#include <cerrno>
#include <limits>
#include <source_location>
#include <type_traits>
#include <vector>

namespace copro {

// globals
constexpr uint8_t DELIMITER_1 = 0xAA;
constexpr uint8_t DELIMITER_2 = 0x55;
constexpr uint8_t ESCAPE = 0xBB;
static int PORT;

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
 * @brief serialize an instance of a trivially-copyable datatype,
 * and add it to a vector
 *
 * @tparam T the datatype
 * @param v the vector to add to
 * @param data the data to serialize
 */
template<typename T>
static void vector_append(std::vector<uint8_t>& v, const T& data) {
    static_assert(std::is_trivially_copyable_v<T>,
                  "Type must be trivially copyable");
    const auto raw = std::bit_cast<std::array<const uint8_t, sizeof(T)>>(data);
    for (uint8_t b : raw) {
        v.push_back(b);
    }
}

std::expected<std::vector<uint8_t>, CoproError>
write_and_receive(MessageId id,
                  const std::vector<uint8_t>& data,
                  int timeout) noexcept {
    // prepare data
    std::vector<uint8_t> out;
    out.push_back(static_cast<uint8_t>(id));
    out.insert(out.end(), data.begin(), data.end());

    // write data
    {
        auto rtn = copro::write(out);
        if (!rtn.has_value()) {
            rtn.error().where.push_back(std::source_location::current());
            return std::unexpected(rtn.error());
        }
    }

    // wait for a response
    const int start = pros::millis();
    while (pros::millis() > start + timeout) {
        auto raw = copro::read();

        // if the read was valid, return it
        if (raw.has_value()) {
            std::vector<uint8_t> rtn;
            rtn.insert(rtn.end(), raw.value().begin() + 1, raw.value().end());
            return rtn;
        }

        // if the read was not successful, we can retry if there's no data
        if (raw.error().type == CoproError::Type::NoData) continue;

        // otherwise, return the error
        raw.error().where.push_back(std::source_location::current());
        return std::unexpected(raw.error());
    }

    // if we're here, then we didn't receive a response within the timeout
    return std::unexpected(CoproError {
      .type = CoproError::Type::TimedOut,
      .what = "timed out while trying to read response!",
      .where = { std::source_location::current() },
    });
}

static CoproError errno_to_copro() {
    CoproError::Type type;
    std::string what;
    switch (errno) {
        case EINVAL:
            type = CoproError::Type::InvalidPort;
            what = std::format(
              "Invalid Port! Expected value between 1-21 (inclusive), but found {}",
              PORT);
            break;
        case EACCES:
            type = CoproError::Type::MultipleAccess;
            what = "Multiple resources trying to access pros serial port API!";
            break;
        case EIO:
            type = CoproError::Type::BrainIoError;
            what = "Brain-side IO error (Vex SDK error)";
        default:
            type = CoproError::Type::Unknown;
            what = "Unknown error occurred";
    }
    return CoproError { .type = type, .what = what, .where = {} };
}

std::expected<void, CoproError> init(int _port, int baud) noexcept {
    // init static vars
    PORT = _port;
    // enable serial port
    if (pros::c::serial_enable(PORT) == PROS_ERR) {
        CoproError err = errno_to_copro();
        err.where.push_back(std::source_location::current());
        return std::unexpected(err);
    }
    pros::delay(50);

    // set serial port baud rate
    if (pros::c::serial_set_baudrate(PORT, baud) == PROS_ERR) {
        CoproError err = errno_to_copro();
        err.where.push_back(std::source_location::current());
        return std::unexpected(err);
    }
    pros::delay(50);

    // flush serial buffer
    if (pros::c::serial_flush(PORT) == PROS_ERR) {
        CoproError err = errno_to_copro();
        err.where.push_back(std::source_location::current());
        return std::unexpected(err);
    }
    pros::delay(10);

    // ping coprocessor until we get a response
    while (true) {
        auto rtn = write_and_receive(MessageId::Ping, {}, 10);
        if (rtn.has_value()) {
            break;
        } else { // error occurred
            auto type = rtn.error().type;
            if (type != CoproError::Type::TimedOut &&
                type != CoproError::Type::NoData &&
                type != CoproError::Type::CorruptedRead &&
                type != CoproError::Type::DataCutOff) {
                rtn.error().where.push_back(std::source_location::current());
                return std::unexpected(rtn.error());
            }
        }
    }

    return {}; // everything OK
}

// protocol:
// [DELIMITER_1 DELIMITER_2] [16-bit length] [CRC16] [stuffed
// payload]
std::expected<void, CoproError>
write(const std::vector<uint8_t>& message) noexcept {
    // stuff the message
    std::vector<uint8_t> payload;
    for (uint8_t b : message) {
        if (b == DELIMITER_1 || b == DELIMITER_2 || b == ESCAPE) {
            payload.push_back(ESCAPE);
        }
        payload.push_back(b);
    }

    // check payload size
    if (payload.size() > std::numeric_limits<uint16_t>::max()) {
        return std::unexpected(CoproError {
          .type = CoproError::Type::MessageTooBig,
          .what =
            "message too big to send to coprocessor, exceeds max value of length byte",
          .where = { std::source_location::current() } });
    }

    // create the header
    std::vector<uint8_t> header;
    // add the delimiter
    header.push_back(DELIMITER_1);
    header.push_back(DELIMITER_2);
    // add the length
    vector_append(header, static_cast<uint16_t>(payload.size()));
    // add the CRC16
    vector_append(header, crc16(message));

    // write header
    if (pros::c::serial_write(PORT, header.data(), header.size()) == PROS_ERR) {
        CoproError err = errno_to_copro();
        err.where.push_back(std::source_location::current());
        return std::unexpected(err);
    }

    // write payload
    if (pros::c::serial_write(PORT, payload.data(), payload.size()) ==
        PROS_ERR) {
        CoproError err = errno_to_copro();
        err.where.push_back(std::source_location::current());
        return std::unexpected(err);
    }

    // OK
    return {};
}

[[nodiscard]]
static std::expected<uint8_t, CoproError> peek_byte() noexcept {
    // up to 2 attempts
    for (int i = 0; i < 2; i++) {
        const auto raw = pros::c::serial_peek_byte(PORT);

        // check for pros error
        // if this error ever occurs, there's no point in retrying
        if (raw == PROS_ERR) {
            CoproError err = errno_to_copro();
            err.where.push_back(std::source_location::current());
            return std::unexpected(err);
        }

        // return if we have data
        if (raw != -1) {
            return static_cast<uint8_t>(raw);
        }

        // small delay to allow new data to arrive
        pros::delay(1);
    }

    // if we're here, then we never received data
    pros::c::serial_flush(PORT);
    return std::unexpected(CoproError {
      .type = CoproError::Type::DataCutOff,
      .what = "serial stream suddenly stopped. Copro disconnect?",
      .where = { std::source_location::current() },
    });
}

[[nodiscard]]
static std::expected<uint8_t, CoproError> read_byte() noexcept {
    // up to 2 attempts
    for (int i = 0; i < 2; i++) {
        const auto raw = pros::c::serial_read_byte(PORT);

        // check for pros error
        // if this error ever occurs, there's no point in retrying
        if (raw == PROS_ERR) {
            CoproError err = errno_to_copro();
            err.where.push_back(std::source_location::current());
            return std::unexpected(err);
        }

        // return if we have data
        if (raw != -1) {
            return static_cast<uint8_t>(raw);
        }

        // small delay to allow new data to arrive
        pros::delay(1);
    }

    // if we're here, then we never received data
    pros::c::serial_flush(PORT);
    return std::unexpected(CoproError {
      .type = CoproError::Type::TimedOut,
      .what = "serial stream suddenly stopped. Copro disconnect?",
      .where = { std::source_location::current() },
    });
}

template<typename T>
static std::expected<T, CoproError> read_stream() {
    static_assert(std::is_trivially_copyable_v<T>,
                  "Type must be trivially copyable");

    // get raw data from serial stream
    std::array<uint8_t, sizeof(T)> raw;
    for (uint8_t& b : raw) {
        auto in = read_byte();

        // check for errors
        if (!in.has_value()) {
            in.error().where.push_back(std::source_location::current());
            return std::unexpected(in.error());
        }
    }

    // cast the raw data to T and return it
    return std::bit_cast<T>(raw);
}

// protocol:
// [DELIMITER_1 DELIMITER_2] [16-bit length] [CRC16] [stuffed
// payload]
std::expected<std::vector<uint8_t>, CoproError> read() noexcept {
    // check if there's data available
    const int avail = pros::c::serial_get_read_avail(PORT);
    if (avail == 0) {
        return std::unexpected(CoproError {
          .type = CoproError::Type::NoData,
          .what = "tried reading serial stream, but no data present!",
          .where = { std::source_location::current() } });
    }
    if (avail == PROS_ERR) {
        CoproError err = errno_to_copro();
        err.where.push_back(std::source_location::current());
        return std::unexpected(err);
    }

    // find the next delimiter
    while (true) {
        // find DELIMITER_1
        if (read_byte() != DELIMITER_1) {
            continue;
        }
        // if the next byte is DELIMITER_2, we've found the
        // delimiter
        if (read_byte() == DELIMITER_2) {
            break;
        }
    }

    // read the header
    const auto length = read_stream<uint16_t>();
    const auto crc = read_stream<uint16_t>();

    // check for errors
    if (!length.has_value()) {
        auto err = length.error();
        err.where.push_back(std::source_location::current());
    }
    if (!crc.has_value()) {
        auto err = crc.error();
        err.where.push_back(std::source_location::current());
    }

    // read the payload
    std::vector<uint8_t> payload;
    for (int i = 0; i < length.value(); ++i) {
        auto b = peek_byte();
        // check for errors from peek_byte
        if (!b.has_value()) {
            b.error().where.push_back(std::source_location::current());
            return std::unexpected(b.error());
        }

        // if the next byte is DELIMITER_1 or DELIMITER_2, bail
        if (b == DELIMITER_1 || b == DELIMITER_2) {
            return std::unexpected(
              CoproError { .type = CoproError::Type::CorruptedRead,
                           .what = "Unescaped delimiter found!",
                           .where = { std::source_location::current() } });

        } else { // otherwise, pop it from the serial buffer
            auto x = read_byte();
            if (!x.has_value()) {
                x.error().where.push_back(std::source_location::current());
                return std::unexpected(x.error());
            }
        }

        // if an escape is found, read the next byte
        if (b == ESCAPE) {
            ++i;
            auto in = read_byte();
            // check for errors
            if (!in.has_value()) {
                in.error().where.push_back(std::source_location::current());
                return std::unexpected(in.error());
            }
            payload.push_back(in.value());
        } else {
            payload.push_back(b.value());
        }
    }

    // validate payload with CRC
    if (crc != crc16(payload)) {
        return std::unexpected(
          CoproError { .type = CoproError::Type::CorruptedRead,
                       .what = "CRC check failed!",
                       .where = { std::source_location::current() } });
    }

    // return payload
    return payload;
}
} // namespace copro
