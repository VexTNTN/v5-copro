#pragma once

#include "pros/rtos.hpp"
#include <algorithm>
#include <array>
#include <bit>
#include <cstdint>
#include <expected>
#include <iostream>
#include <source_location>
#include <string>
#include <vector>

namespace copro {

enum class MessageId : uint8_t {
    Ping = 0,
    GetOtosStatus = 1,
    OtosSelfTest = 24,
    OtosResetTracking = 3,
    OtosGetPose = 7,
    OtosSetPose = 8,
    OtosGetAccel = 12,
    OtosGetLinearScalar = 18,
    OtosSetLinearScalar = 19,
    OtosGetAngularScalar = 20,
    OtosSetAngularScalar = 21,
    OtosCalibrate = 25,
    OtosIsCalibrated = 26
};

struct CoproError {
    enum class Type {
        Unknown,
        InvalidPort,
        NoDevice,
        MultipleAccess,
        TimedOut,
        MessageTooBig,
        BrainIoError,
        NoData,
        CorruptedRead,
        DataCutOff,
        InvalidBaud,
    } type;

    std::string what;
    std::vector<std::source_location> where;
};

// Enable printing CoproError using std::cout
std::ostream& operator<<(std::ostream& os, const CoproError& err);

void print_error(CoproError& error,
                 std::source_location loc = std::source_location::current());

class Coprocessor {
  public:
    /**
     * @brief Create a Coprocessor instance.
     * @param port the smart port number
     * @param baud the baud rate (default 921600 recommended)
     */
    constexpr Coprocessor(uint8_t port, uint32_t baud = 921600)
        : m_port(port),
          m_baud(baud) {};

    // Delete copy to prevent hardware resource contention issues
    Coprocessor(const Coprocessor&) = delete;
    Coprocessor& operator=(const Coprocessor&) = delete;

    /**
     * @brief Initialize the coprocessor serial port.
     * Blocks shortly. Does not perform handshake
     */
    [[nodiscard]]
    std::expected<void, CoproError> init();

    /**
     * @brief Write a message to the coprocessor and wait for a response.
     */
    [[nodiscard]]
    std::expected<std::vector<uint8_t>, CoproError>
    write_and_receive(MessageId id,
                      const std::vector<uint8_t>& data,
                      int timeout,
                      bool silence = false);

  private:
    // --- Member Variables ---
    const uint8_t m_port;
    const uint32_t m_baud;
    pros::Mutex m_mutex;

    // --- Constants ---
    static constexpr uint8_t COBS_DELIMITER = 0x00;

    // --- Private Implementation Helpers ---

    [[nodiscard]]
    std::expected<std::vector<uint8_t>, CoproError>
    write_and_receive_impl(MessageId id,
                           const std::vector<uint8_t>& data,
                           int timeout);

    // Low-level IO
    std::expected<void, CoproError> write(const std::vector<uint8_t>& message);
    std::expected<std::vector<uint8_t>, CoproError> read();

    // Serial Primitives
    std::expected<uint8_t, CoproError> serial_byte_op(bool peek);
    std::expected<uint8_t, CoproError> read_byte();
    std::expected<uint8_t, CoproError> peek_byte();
    std::expected<std::vector<uint8_t>, CoproError> read_exact(size_t n);

    // Static Pure Logic Helpers
    static uint16_t crc16(const std::vector<uint8_t>& data);
    static std::vector<uint8_t> cobs_encode(const std::vector<uint8_t>& data);
    static std::vector<uint8_t> cobs_decode(const std::vector<uint8_t>& data);

    static CoproError make_errno_error(
      int current_port,
      std::source_location loc = std::source_location::current());
    static CoproError trace_error(CoproError err, std::source_location loc);

    // --- Template Helpers ---

    template<typename T>
        requires std::is_trivially_copyable_v<T>
    static std::vector<uint8_t> serialize(const T& data) {
        auto raw = std::bit_cast<std::array<uint8_t, sizeof(T)>>(data);
        return { raw.begin(), raw.end() };
    }

    template<typename T>
        requires std::is_trivially_copyable_v<T>
    std::expected<T, CoproError> read_pod() {
        auto bytes_res = read_exact(sizeof(T));
        if (!bytes_res) return std::unexpected(bytes_res.error());

        std::array<uint8_t, sizeof(T)> raw;
        std::copy(bytes_res->begin(), bytes_res->end(), raw.begin());
        return std::bit_cast<T>(raw);
    }
};

} // namespace copro
