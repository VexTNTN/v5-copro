#pragma once

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <expected>
#include <source_location>
#include <string>
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
    requires std::is_trivially_copyable_v<T>
static std::vector<uint8_t> serialize(const T& data) {
    auto raw = std::bit_cast<std::array<uint8_t, sizeof(T)>>(data);
    return { raw.begin(), raw.end() };
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
template<typename T>
    requires std::is_trivially_copyable_v<T>
static T deserialize(const std::vector<uint8_t>& data) {
    // Ensure we don't read out of bounds (basic safety check)
    if (data.size() < sizeof(T)) return T {};

    std::array<uint8_t, sizeof(T)> raw;
    std::copy_n(data.begin(), sizeof(T), raw.begin());
    return std::bit_cast<T>(raw);
}

enum class MessageId : uint8_t {
    Ping = 0,
    GetOtosStatus = 1,
    OtosSelfTest = 24,
    OtosResetTracking = 3,
    OtosGetPose = 7,
    OtosSetPose = 8,
    OtosGetAccel = 12,
    OtosSetOffset = 28,
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

// enable printing CoproError using std::cout
std::ostream& operator<<(std::ostream& os, const CoproError& err);

class Coprocessor {
  public:
    /**
     * @brief Create a Coprocessor instance on a port with a specific baud rate.
     * Non-blocking.
     *
     * @note baud rate must be 9600, 19200, 38400, 57600, 115200, 230400,
     * 460800, or 921600
     *
     * @param port the smart port the Coprocessor is on
     * @param baud the baud rate the serial stream should run on
     */
    constexpr Coprocessor(uint8_t port, uint32_t baud) noexcept
        : port(port),
          baud(baud) {};

    /**
     * @brief initialize the coprocessor
     *
     * This function configures a smart port to be a generic serial port,
     * and performs a simple handshake with the coprocessor. It blocks
     * until the handshake is complete
     *
     * @param port
     * @param baud
     * @return std::expected<void, CoproError>
     */
    [[nodiscard]]
    std::expected<void, CoproError> init(int port, int baud) noexcept;

    [[nodiscard]]
    bool is_initialized() noexcept;

    /**
     * @brief Write a message to the coprocessor and wait for a response
     *
     * @param id the message ID
     * @param data the payload data
     * @param timeout how long to wait for a response, in milliseconds
     *
     * @return std::vector<uint8_t> the response payload on success
     * @return CoproError on failure
     */
    [[nodiscard]]
    std::expected<std::vector<uint8_t>, CoproError>
    write_and_receive(MessageId id,
                      const std::vector<uint8_t>& data,
                      int timeout,
                      bool silence = false) noexcept;

    constexpr ~Coprocessor() = delete ("Should never need to be destroyed");
    constexpr Coprocessor(Coprocessor&) =
      delete ("should never need to be copied");
    constexpr Coprocessor(const Coprocessor&) =
      delete ("should never beed to be copied");

  private:
    [[nodiscard]]
    std::expected<std::vector<uint8_t>, CoproError>
    write_and_receive_impl(MessageId id,
                           const std::vector<uint8_t>& data,
                           int timeout) noexcept;
    std::atomic<bool> initialized;
    uint8_t port;
    const uint32_t baud;
};
} // namespace copro
