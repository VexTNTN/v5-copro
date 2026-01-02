#include "Coprocessor.hpp"
#include "pros/error.h"
#include "pros/rtos.hpp"
#include "pros/serial.h"
#include <cerrno>
#include <format>

namespace copro {

namespace { // Internal linkage

constexpr uint8_t DELIMITER_1 = 0xAA;
constexpr uint8_t DELIMITER_2 = 0x55;
constexpr uint8_t ESCAPE = 0xBB;
static int s_port; // Renamed to denote static

// --- Error Handling Helpers ---

// Helper to convert errno/PROS errors to CoproError
CoproError
make_errno_error(std::source_location loc = std::source_location::current()) {
    CoproError::Type type;
    std::string what;

    switch (errno) {
        case EINVAL:
            type = CoproError::Type::InvalidPort;
            what = std::format("Invalid Port! Found {}", s_port);
            break;
        case EACCES:
            type = CoproError::Type::MultipleAccess;
            what = "Port access conflict";
            break;
        case EIO:
            type = CoproError::Type::BrainIoError;
            what = "Vex SDK Brain IO Error";
            break;
        default: type = CoproError::Type::Unknown; what = "Unknown error";
    }
    return CoproError { type, std::move(what), { loc } };
}

// Helper to append source location to an existing error
CoproError trace_error(CoproError err, std::source_location loc) {
    err.where.push_back(loc);
    return err;
}

// Helper to conditionally move the value only if it isn't void
template<typename T, typename E>
constexpr auto unwrap(std::expected<T, E>& expected) {
    if constexpr (std::is_void_v<T>) {
        return; // Do nothing for void
    } else {
        return std::move(*expected); // Move the value for non-void
    }
}

// MACRO: Unwraps std::expected. If error, returns unexpected.
// Now uses 'unwrap' to safely handle void types.
#define TRY(expr)                                            \
    ({                                                       \
        auto _res = (expr);                                  \
        if (!_res.has_value()) {                             \
            return std::unexpected(                          \
              trace_error(std::move(_res.error()),           \
                          std::source_location::current())); \
        }                                                    \
        unwrap(_res);                                        \
    })

// --- CRC16 ---
uint16_t crc16(const std::vector<uint8_t>& data) {
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

// --- Serial Primitives ---

// Checks if a PROS API call returned error, if so, returns unexpected
std::expected<void, CoproError> check_pros(int result) {
    if (result == PROS_ERR) {
        return std::unexpected(make_errno_error());
    }
    return {};
}

// Consolidated logic for read/peek
std::expected<uint8_t, CoproError> serial_byte_op(bool peek) {
    for (int i = 0; i < 2; i++) {
        int raw = peek ? pros::c::serial_peek_byte(s_port) :
                         pros::c::serial_read_byte(s_port);

        if (raw == PROS_ERR) return std::unexpected(make_errno_error());
        if (raw != -1) return static_cast<uint8_t>(raw);

        // wait for new data
        pros::delay(1);
    }

    return std::unexpected(
      CoproError { .type = CoproError::Type::DataCutOff,
                   .what = "Serial stream cut off. Disconnection?",
                   .where = { std::source_location::current() } });
}

std::expected<uint8_t, CoproError> read_byte() {
    return serial_byte_op(false);
}

std::expected<uint8_t, CoproError> peek_byte() {
    return serial_byte_op(true);
}

// Reads exactly N bytes.
std::expected<std::vector<uint8_t>, CoproError> read_exact(size_t n) {
    std::vector<uint8_t> out;
    out.reserve(n);
    for (size_t i = 0; i < n; ++i) {
        out.push_back(TRY(read_byte()));
    }
    return out;
}

// Reads a POD type
template<typename T>
std::expected<T, CoproError> read_pod() {
    static_assert(std::is_trivially_copyable_v<T>);
    auto bytes = TRY(read_exact(sizeof(T)));
    std::array<uint8_t, sizeof(T)> raw;
    std::copy(bytes.begin(), bytes.end(), raw.begin());
    return std::bit_cast<T>(raw);
}

} // anonymous namespace

// --- Public Implementation ---

std::expected<void, CoproError> init(int port, int baud) noexcept {
    s_port = port;

    TRY(check_pros(pros::c::serial_enable(s_port)));
    pros::delay(50);

    TRY(check_pros(pros::c::serial_set_baudrate(s_port, baud)));
    pros::delay(50);

    TRY(check_pros(pros::c::serial_flush(s_port)));
    pros::delay(10);

    // Ping until response
    while (true) {
        auto rtn = write_and_receive(MessageId::Ping, {}, 10);
        if (rtn) break;

        // Only retry on specific communication errors
        auto type = rtn.error().type;
        bool can_retry = (type == CoproError::Type::TimedOut ||
                          type == CoproError::Type::NoData ||
                          type == CoproError::Type::CorruptedRead ||
                          type == CoproError::Type::DataCutOff);

        if (!can_retry) {
            return std::unexpected(
              trace_error(rtn.error(), std::source_location::current()));
        }
    }

    return {};
}

std::expected<void, CoproError>
write(const std::vector<uint8_t>& message) noexcept {
    // 1. Stuff Payload
    std::vector<uint8_t> payload;
    payload.reserve(message.size() + 8); // Heuristic reservation

    for (uint8_t b : message) {
        if (b == DELIMITER_1 || b == DELIMITER_2 || b == ESCAPE) {
            payload.push_back(ESCAPE);
        }
        payload.push_back(b);
    }

    if (payload.size() > std::numeric_limits<uint16_t>::max()) {
        return std::unexpected(
          CoproError { .type = CoproError::Type::MessageTooBig,
                       .what = "Payload exceeds max length",
                       .where = { std::source_location::current() } });
    }

    // 2. Build Header
    std::vector<uint8_t> packet;
    packet.reserve(payload.size() + 6);
    packet.push_back(DELIMITER_1);
    packet.push_back(DELIMITER_2);

    auto len_bytes = serialize(static_cast<uint16_t>(payload.size()));
    packet.insert(packet.end(), len_bytes.begin(), len_bytes.end());

    auto crc_bytes =
      serialize(crc16(message)); // CRC computed on *original* message
    packet.insert(packet.end(), crc_bytes.begin(), crc_bytes.end());

    // 3. Send Header + Payload (Split writes to match original logic)
    TRY(
      check_pros(pros::c::serial_write(s_port, packet.data(), packet.size())));
    TRY(check_pros(
      pros::c::serial_write(s_port, payload.data(), payload.size())));

    return {};
}

std::expected<std::vector<uint8_t>, CoproError> read() noexcept {
    int avail = pros::c::serial_get_read_avail(s_port);
    if (avail == PROS_ERR) return std::unexpected(make_errno_error());
    if (avail == 0) {
        return std::unexpected(
          CoproError { CoproError::Type::NoData,
                       "No data",
                       { std::source_location::current() } });
    }

    // 1. Sync to delimiters
    while (true) {
        if (TRY(read_byte()) == DELIMITER_1) {
            if (TRY(read_byte()) == DELIMITER_2) break;
        }
    }

    // 2. Read Header
    uint16_t length = TRY(read_pod<uint16_t>());
    uint16_t expected_crc = TRY(read_pod<uint16_t>());

    // 3. Read Raw Stuffed Payload
    // The length on wire is the *stuffed* length
    std::vector<uint8_t> raw_payload = TRY(read_exact(length));

    // 4. Unstuff
    std::vector<uint8_t> data;
    data.reserve(length); // Will be <= raw length

    for (size_t i = 0; i < raw_payload.size(); ++i) {
        uint8_t b = raw_payload[i];

        // Sanity check: Inner delimiters shouldn't exist unless escaped
        if (b == DELIMITER_1 || b == DELIMITER_2) {
            return std::unexpected(
              CoproError { CoproError::Type::CorruptedRead,
                           "Unescaped delimiter found",
                           { std::source_location::current() } });
        }

        if (b == ESCAPE) {
            if (++i >= raw_payload.size()) {
                return std::unexpected(
                  CoproError { CoproError::Type::CorruptedRead,
                               "Trailing escape byte",
                               { std::source_location::current() } });
            }
            data.push_back(raw_payload[i]);
        } else {
            data.push_back(b);
        }
    }

    // 5. Verify CRC
    if (crc16(data) != expected_crc) {
        return std::unexpected(
          CoproError { CoproError::Type::CorruptedRead,
                       "CRC Mismatch",
                       { std::source_location::current() } });
    }

    return data;
}

std::expected<std::vector<uint8_t>, CoproError>
write_and_receive(MessageId id,
                  const std::vector<uint8_t>& data,
                  int timeout) noexcept {

    // Prepare packet: [ID] [DATA...]
    std::vector<uint8_t> packet;
    packet.reserve(data.size() + 1);
    packet.push_back(static_cast<uint8_t>(id));
    packet.insert(packet.end(), data.begin(), data.end());

    TRY(write(packet));

    const int start = pros::millis();
    while (pros::millis() < start + timeout) {
        auto raw = read();
        if (raw.has_value()) {
            // Strip the ID (first byte) from the response
            if (raw->empty())
                return {}; // Should not happen based on protocol, but safety
                           // first
            return std::vector<uint8_t>(raw->begin() + 1, raw->end());
        }

        if (raw.error().type == CoproError::Type::NoData) {
            pros::delay(1); // Yield to prevent tight loop burning CPU
            continue;
        }

        return std::unexpected(
          trace_error(raw.error(), std::source_location::current()));
    }

    return std::unexpected(CoproError { CoproError::Type::TimedOut,
                                        "Response timeout",
                                        { std::source_location::current() } });
}

} // namespace copro
