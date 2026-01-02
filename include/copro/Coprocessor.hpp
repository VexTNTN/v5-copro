#pragma once

#include <algorithm>
#include <cstdint>
#include <expected>
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
        MultipleAccess,
        TimedOut,
        MessageTooBig,
        BrainIoError,
        NoData,
        CorruptedRead,
        DataCutOff,
    } type;

    std::string what;
    std::vector<std::source_location> where;
};

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

/**
 * @brief initialize comms with the coprocessor.
 *
 * @warning this function is blocking
 *
 * @param port the port the coprocessor is on
 * @param baud the baud rate of the serial port. Usually 921600
 * @param timeout max time that can be taken to initialize
 *
 * @return void on success
 * @return CoproError on failure
 */
[[nodiscard]]
std::expected<void, CoproError> init(int _port, int baud) noexcept;

/**
 * @brief Write a vector of bytes of the serial port
 *
 * @param message vector of bytes to write
 *
 * @return void on success
 * @return CoproError on failure
 */
[[nodiscard]]
std::expected<void, CoproError>
write(const std::vector<uint8_t>& message) noexcept;

/**
 * @brief Read a packet, parse it, and return the message
 *
 * @return the payload on success
 * @return CoproError on failure
 */
[[nodiscard]]
std::expected<std::vector<uint8_t>, CoproError> read() noexcept;

/**
 * @brief Write a message to the coprocessor and wait for a response
 *
 * @param id the message ID
 * @param data the payload data
 * @param timeout how long to wait for a response, in milliseconds
 *
 * @return std::vector<uint8_t> the response payload on success
 * @return std::vector<uint8_t> CoproError on failure
 */
[[nodiscard]]
std::expected<std::vector<uint8_t>, CoproError>
write_and_receive(MessageId id,
                  const std::vector<uint8_t>& data,
                  int timeout) noexcept;
} // namespace copro
