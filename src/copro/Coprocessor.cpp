#include "Coprocessor.hpp"
#include "pros/error.h"
#include "pros/rtos.hpp"
#include "pros/serial.h"
#include <cerrno>
#include <format>
#include <mutex>
#include <ranges>

namespace copro {

// --- Private Logic Helpers (Internal Linkage) ---

namespace {

// Helper to conditionally move the value only if it isn't void
template<typename T, typename E>
constexpr auto unwrap(std::expected<T, E>& expected) {
    if constexpr (std::is_void_v<T>) {
        return;
    } else {
        return std::move(*expected);
    }
}

// Checks if a PROS API call returned error
std::expected<void, CoproError> check_pros(int result, int port) {
    if (result == PROS_ERR) {
        // We cannot access Coprocessor::make_errno_error easily here without
        // making it public or duplicating logic.
        // For simplicity, we implement a quick mapping or just pass errno.
        // However, to keep the class clean, let's just return void error
        // and let the caller handle the errno creation using the class method.
        return std::unexpected(
          CoproError { CoproError::Type::BrainIoError,
                       "PROS Error",
                       { std::source_location::current() } });
    }
    return {};
}

} // anonymous namespace

// Defines for the TRY macro to work within member functions
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

// --- CoproError Implementation ---

std::ostream& operator<<(std::ostream& os, const CoproError& err) {
    os << "[CoproError] Type: ";
    switch (err.type) {
        case CoproError::Type::Unknown: os << "Unknown"; break;
        case CoproError::Type::InvalidPort: os << "InvalidPort"; break;
        case CoproError::Type::NoDevice: os << "NoDevice"; break;
        case CoproError::Type::MultipleAccess: os << "MultipleAccess"; break;
        case CoproError::Type::TimedOut: os << "TimedOut"; break;
        case CoproError::Type::MessageTooBig: os << "MessageTooBig"; break;
        case CoproError::Type::BrainIoError: os << "BrainIoError"; break;
        case CoproError::Type::NoData: os << "NoData"; break;
        case CoproError::Type::CorruptedRead: os << "CorruptedRead"; break;
        case CoproError::Type::DataCutOff: os << "DataCutOff"; break;
        case CoproError::Type::InvalidBaud: os << "InvalidBaud"; break;
        default: os << "Unknown(" << (int)err.type << ")"; break;
    }
    os << " | Msg: \"" << err.what << "\"\n";

    if (!err.where.empty()) {
        os << "  Trace:\n";
        // Iterate in reverse
        for (const auto& loc : std::views::reverse(err.where)) {
            os << "    at " << loc.function_name() << " (" << loc.file_name()
               << ":" << loc.line() << ":" << loc.column() << ":"
               << loc.column() << ")\n";
        }
    }
    return os;
}

void print_error(CoproError& error, std::source_location loc) {
    error.where.push_back(loc);
    std::cout << error << std::endl;
}

// --- Coprocessor Class Implementation ---

CoproError Coprocessor::make_errno_error(int port, std::source_location loc) {
    CoproError::Type type;
    std::string what;

    switch (errno) {
        case EINVAL:
            type = CoproError::Type::InvalidPort;
            what = std::format("Invalid Port! Found {}", port);
            break;
        case EACCES:
            type = CoproError::Type::MultipleAccess;
            what = "Port access conflict";
            break;
        case EIO:
            type = CoproError::Type::BrainIoError;
            what = "Vex SDK Brain IO Error";
            break;
        case ENODEV:
            type = CoproError::Type::NoDevice;
            what = "Port not initialized as pros generic serial";
            break;
        case EADDRINUSE:
            type = CoproError::Type::InvalidPort;
            what = "Invalid port! Non-V5 device present";
            break;
        default:
            type = CoproError::Type::Unknown;
            what = "Unknown error";
            break;
    }
    return CoproError { type, std::move(what), { loc } };
}

CoproError Coprocessor::trace_error(CoproError err, std::source_location loc) {
    err.where.push_back(loc);
    return err;
}

uint16_t Coprocessor::crc16(const std::vector<uint8_t>& data) {
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

std::vector<uint8_t>
Coprocessor::cobs_encode(const std::vector<uint8_t>& data) {
    std::vector<uint8_t> result;
    result.reserve(data.size() + data.size() / 254 + 2);

    size_t code_index = result.size();
    result.push_back(0); // placeholder for code
    uint8_t code = 1;

    for (uint8_t byte : data) {
        if (byte != 0) {
            result.push_back(byte);
            code++;
            if (code == 0xFF) {
                result.at(code_index) = code;
                code = 1;
                code_index = result.size();
                result.push_back(0);
            }
        } else {
            result.at(code_index) = code;
            code = 1;
            code_index = result.size();
            result.push_back(0);
        }
    }
    result.at(code_index) = code;
    return result;
}

std::vector<uint8_t>
Coprocessor::cobs_decode(const std::vector<uint8_t>& data) {
    std::vector<uint8_t> decoded;
    decoded.reserve(data.size());
    size_t i = 0;
    while (i < data.size()) {
        uint8_t code = data[i++];
        if (code == 0) break; // Should not happen in valid COBS frame

        for (uint8_t j = 1; j < code; j++) {
            if (i >= data.size()) break;
            decoded.push_back(data[i++]);
        }
        if (code < 0xFF && i < data.size()) {
            decoded.push_back(0);
        }
    }
    return decoded;
}

std::expected<void, CoproError> Coprocessor::init() noexcept {
    // Note: m_port is now used instead of s_port
    int res = pros::c::serial_enable(m_port);
    if (res == PROS_ERR) return std::unexpected(make_errno_error(m_port));
    pros::delay(50);

    res = pros::c::serial_set_baudrate(m_port, m_baud);
    if (res == PROS_ERR) return std::unexpected(make_errno_error(m_port));
    pros::delay(50);

    res = pros::c::serial_flush(m_port);
    if (res == PROS_ERR) return std::unexpected(make_errno_error(m_port));
    pros::delay(10);

    return {};
}

std::expected<uint8_t, CoproError> Coprocessor::serial_byte_op(bool peek) {
    for (int i = 0; i < 2; i++) {
        int raw = peek ? pros::c::serial_peek_byte(m_port) :
                         pros::c::serial_read_byte(m_port);

        if (raw == PROS_ERR) return std::unexpected(make_errno_error(m_port));
        if (raw != -1) return static_cast<uint8_t>(raw);

        // wait for new data
        pros::delay(1);
    }

    return std::unexpected(
      CoproError { .type = CoproError::Type::DataCutOff,
                   .what = "Serial stream cut off. Disconnection?",
                   .where = { std::source_location::current() } });
}

std::expected<uint8_t, CoproError> Coprocessor::read_byte() {
    return serial_byte_op(false);
}

std::expected<uint8_t, CoproError> Coprocessor::peek_byte() {
    return serial_byte_op(true);
}

std::expected<std::vector<uint8_t>, CoproError>
Coprocessor::read_exact(size_t n) {
    std::vector<uint8_t> out;
    out.reserve(n);
    for (size_t i = 0; i < n; ++i) {
        out.push_back(TRY(read_byte()));
    }
    return out;
}

std::expected<void, CoproError>
Coprocessor::write(const std::vector<uint8_t>& message) noexcept {
    // 1. Build Payload [CRC][Message]
    std::vector<uint8_t> payload;
    payload.reserve(message.size() + 2);

    auto crc_bytes = serialize(crc16(message));
    payload.insert(payload.end(), crc_bytes.begin(), crc_bytes.end());
    payload.insert(payload.end(), message.begin(), message.end());

    // 2. COBS Encode
    std::vector<uint8_t> encoded = cobs_encode(payload);

    // 3. Send [0x00][Encoded][0x00]
    std::vector<uint8_t> packet;
    packet.reserve(encoded.size() + 2);
    packet.push_back(COBS_DELIMITER);
    packet.insert(packet.end(), encoded.begin(), encoded.end());
    packet.push_back(COBS_DELIMITER);

    if (pros::c::serial_write(m_port, packet.data(), packet.size()) == PROS_ERR)
        return std::unexpected(make_errno_error(m_port));

    return {};
}

std::expected<std::vector<uint8_t>, CoproError> Coprocessor::read() noexcept {
    int avail = pros::c::serial_get_read_avail(m_port);
    if (avail == PROS_ERR) return std::unexpected(make_errno_error(m_port));
    // Do not return NoData immediately; allow read_byte loop to try at least
    // once or rely on the caller timeout loop.

    std::vector<uint8_t> buffer;

    // 1. Read until Delimiter (Frame Sync)
    while (true) {
        // We use read_byte which has a small retry loop.
        // If it returns error (timeout/cutoff), we abort.
        uint8_t b = TRY(read_byte());

        if (b == COBS_DELIMITER) {
            if (buffer.empty()) {
                // Found a delimiter but buffer is empty (e.g. leading zero or
                // double zero) Continue waiting for data.
                continue;
            } else {
                // End of Frame
                break;
            }
        }
        buffer.push_back(b);
    }

    // 2. Decode
    std::vector<uint8_t> decoded = cobs_decode(buffer);
    if (decoded.size() < 2) {
        return std::unexpected(
          CoproError { CoproError::Type::CorruptedRead,
                       "Frame too short",
                       { std::source_location::current() } });
    }

    // 3. Verify CRC
    // CRC is first 2 bytes
    uint16_t received_crc = (uint16_t(decoded[1]) << 8) | decoded[0];
    // ^ Assuming little-endian read_pod equivalent logic:
    // Actually, let's match the `write` logic: serialize() uses host order
    // (bit_cast). So we should read it back similarly.
    std::array<uint8_t, 2> crc_raw = { decoded[0], decoded[1] };
    received_crc = std::bit_cast<uint16_t>(crc_raw);

    std::vector<uint8_t> payload(decoded.begin() + 2, decoded.end());

    if (crc16(payload) != received_crc) {
        return std::unexpected(
          CoproError { CoproError::Type::CorruptedRead,
                       "CRC Mismatch",
                       { std::source_location::current() } });
    }

    return payload;
}

std::expected<std::vector<uint8_t>, CoproError>
Coprocessor::write_and_receive(MessageId id,
                               const std::vector<uint8_t>& data,
                               int timeout,
                               bool silence) noexcept {
    auto rtn = write_and_receive_impl(id, data, timeout);
    if (!rtn.has_value() && !silence) {
        std::cout << rtn.error() << std::endl;
    }
    return rtn;
}

std::expected<std::vector<uint8_t>, CoproError>
Coprocessor::write_and_receive_impl(MessageId id,
                                    const std::vector<uint8_t>& data,
                                    int timeout) noexcept {
    std::lock_guard lock(m_mutex);

    std::vector<uint8_t> packet;
    packet.reserve(data.size() + 1);
    packet.push_back(static_cast<uint8_t>(id));
    packet.insert(packet.end(), data.begin(), data.end());

    TRY(write(packet));

    const int start = pros::millis();
    while (pros::millis() < start + timeout) {
        auto raw = read();
        if (raw.has_value()) {
            if (raw->empty()) return {};
            return std::vector<uint8_t>(raw->begin() + 1, raw->end());
        }

        // If read() timed out waiting for delimiter (DataCutOff) or found
        // nothing (NoData check removed inside read but error propagates),
        // retry
        if (raw.error().type == CoproError::Type::DataCutOff ||
            raw.error().type == CoproError::Type::NoData) {
            pros::delay(1);
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
