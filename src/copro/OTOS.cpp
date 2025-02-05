#include "copro/OTOS.hpp"
#include "Coprocessor.hpp"
#include <cmath>
#include <limits>

namespace copro {

//////////////////////////////////////
// using
/////////////////

using ErrorType = OTOS::ErrorType;
using _Error = Error<ErrorType>;
using enum ErrorType;
using Status = OTOS::Status;
using Version = OTOS::Version;

//////////////////////////////////////
// macros
/////////////////

// macro to simplify returning an ERROR
#define ERROR(...) std::unexpected<_Error>(_Error(__VA_ARGS__))

// macro to simplify checking write_and_receive
#define CHECK_WRITE(response, num)                                                                                     \
    if (!response) {                                                                                                   \
        return std::unexpected(_Error(response.error(), RS485_IO,                                                      \
                                      "failed to send command to OTOS with id {} on port {}", m_device,                \
                                      m_coprocessor->get_port()));                                                     \
    }                                                                                                                  \
    if (response->size() != num) {                                                                                     \
        return std::unexpected<_Error>(_Error(INCORRECT_RESPONSE_SIZE,                                                 \
                                              "expected response size of {} from OTOS with id {} on port {}", num,     \
                                              m_device, m_coprocessor->get_port()));                                   \
    }

// macro to simplify checking whether the OTOS has been initialized
#define CHECK_INITIALIZE()                                                                                             \
    if (!m_initialized) {                                                                                              \
        return std::unexpected<_Error>(_Error(NOT_INITIALIZED, "OTOS with id {} on port {} has not been initialized",  \
                                              m_device, m_coprocessor->get_port()));                                   \
    }

// macro to simplify returning an I2C_IO error
#define I2CERR                                                                                                         \
    std::unexpected<_Error>(                                                                                           \
        _Error(I2C_IO, "failed to interact with OTOS with id {} on port {}", m_device, m_coprocessor->get_port()));

// macro to simplify returning an unknown error
#define UNKNOWNERR                                                                                                     \
    std::unexpected<_Error>(_Error(UNKNOWN,                                                                            \
                                   "unknown error response when trying to interact with OTOS with id {} on port {}",   \
                                   m_device, m_coprocessor->get_port()));

//////////////////////////////////////
// constants
/////////////////
constexpr Version PROTOCOL_VERSION {
    .major = 0,
    .minor = 1,
};
constexpr int READ_TIMEOUT = 5;

namespace topic {
constexpr const char* INITIALIZE = "otos/initialize";
constexpr const char* GET_HARDWARE_VERSION = "otos/get_hardware_version";
constexpr const char* GET_FIRMWARE_VERSION = "otos/get_firmware_version";
constexpr const char* SELF_TEST_WRITE = "otos/self_test_write";
constexpr const char* SELF_TEST_READ = "otos/self_test_read";
constexpr const char* CALIBRATE_IMU = "otos/calibrate_imu";
constexpr const char* IS_IMU_CALIBRATED = "otos/is_imu_calibrated";
constexpr const char* GET_LINEAR_SCALAR = "otos/get_linear_scalar";
constexpr const char* SET_LINEAR_SCALAR = "otos/set_linear_scalar";
constexpr const char* GET_ANGULAR_SCALAR = "otos/get_angular_scalar";
constexpr const char* SET_ANGULAR_SCALAR = "otos/set_angular_scalar";
constexpr const char* GET_STATUS = "otos/get_status";
constexpr const char* GET_OFFSET = "otos/get_offset";
constexpr const char* SET_OFFSET = "otos/set_offset";
constexpr const char* GET_POSE = "otos/get_pose";
constexpr const char* SET_POSE = "otos/set_pose";
constexpr const char* GET_VELOCITY = "otos/get_velocity";
constexpr const char* GET_ACCELERATION = "otos/get_acceleration";
} // namespace topic

constexpr float RADIAN_TO_DEGREE = 180.0 / 3.14159;
constexpr float METER_TO_INCH = 39.3701;
constexpr float METER_TO_INT16 = 32768.0 / 10.0;
constexpr float INT16_TO_METER = 1.0 / METER_TO_INT16;
constexpr float INT16_TO_INCH = INT16_TO_METER * METER_TO_INCH;
constexpr float INCH_TO_INT16 = 1.0 / INT16_TO_INCH;
constexpr float RAD_TO_INT16 = 32768.0 / 3.14159;
constexpr float INT16_TO_RAD = 1.0 / RAD_TO_INT16;
constexpr float INT16_TO_DEG = INT16_TO_RAD * RADIAN_TO_DEGREE;
constexpr float DEG_TO_INT16 = 1.0 / INT16_TO_DEG;

//////////////////////////////////////
// util
/////////////////

template <typename T> static std::vector<uint8_t> serialize(const T& data)
    requires std::is_trivially_copyable_v<T>
{
    auto raw = std::bit_cast<std::array<uint8_t, sizeof(T)>>(data);
    std::vector<uint8_t> out(sizeof(T));
    for (int i = 0; i < sizeof(T); ++i) out.at(i) = raw.at(i);
    return out;
}

template <typename T, int N> static T deserialize(const std::vector<uint8_t>& data)
    requires std::is_trivially_copyable_v<T>
{
    std::array<uint8_t, N> raw;
    for (int i = 0; i < N; ++i) raw.at(i) = data.at(i);
    return std::bit_cast<T>(raw);
}

//////////////////////////////////////
// OTOS
/////////////////

OTOS::OTOS(std::shared_ptr<copro::Coprocessor> coprocessor, const std::string& device)
    : m_coprocessor(coprocessor),
      m_device(device) {}

std::expected<void, _Error> OTOS::initialize() {
    // check that the OTOS has been initialized
    if (m_initialized) {
        return ERROR(ALREADY_INITIALIZED, "OTOS with id {} on port {} has already been initialized", m_device,
                     m_coprocessor->get_port());
    }
    // prepare data
    std::vector<uint8_t> out;
    for (auto c : m_device) out.push_back(c);
    // send data
    auto raw = m_coprocessor->write_and_receive(topic::INITIALIZE, out, READ_TIMEOUT);
    // check for IO errors
    CHECK_WRITE(raw, 1);
    // check respose
    switch (raw->at(0)) {
        case 0: {
            m_initialized = true;
            return {};
        }
        case 1: {
            m_initialized = true;
            // TODO: add warning here
            return {};
        }
        case 2: return I2CERR;
        default: return UNKNOWNERR;
    }
}

std::expected<Version, _Error> OTOS::get_hardware_version() {
    // check that the OTOS has been initialized
    CHECK_INITIALIZE();
    // send data
    auto raw = m_coprocessor->write_and_receive(topic::GET_HARDWARE_VERSION, {}, READ_TIMEOUT);
    // check for IO errors
    CHECK_WRITE(raw, 2);
    // check respose
    switch (raw->at(0)) {
        case std::numeric_limits<uint8_t>::max(): return I2CERR;
        default: return Version(raw->at(0), raw->at(1));
    }
}

std::expected<Version, _Error> OTOS::get_firmware_version() {
    // check that the OTOS has been initialized
    CHECK_INITIALIZE();
    // send data
    auto raw = m_coprocessor->write_and_receive(topic::GET_FIRMWARE_VERSION, {}, READ_TIMEOUT);
    // check for IO errors
    CHECK_WRITE(raw, 2);
    // check respose
    switch (raw->at(0)) {
        case std::numeric_limits<uint8_t>::max(): return I2CERR;
        default: return Version(raw->at(0), raw->at(1));
    }
}

std::expected<bool, Error<ErrorType>> OTOS::self_test() {
    union {
            struct {
                    /// @brief Write 1 to start the self test
                    uint8_t start : 1;

                    /// @brief Returns 1 while the self test is in progress
                    uint8_t inProgress : 1;

                    /// @brief Returns 1 if the self test passed
                    uint8_t pass : 1;

                    /// @brief Returns 1 if the self test failed
                    uint8_t fail : 1;

                    /// @brief Reserved bits, do not use
                    uint8_t reserved : 4;
            };

            /// @brief Raw register value
            uint8_t value;
    } res;

    // check that the OTOS has been initialized
    CHECK_INITIALIZE();
    // send self test register
    auto raw = m_coprocessor->write_and_receive(topic::SELF_TEST_WRITE, {}, READ_TIMEOUT);
    // check for IO errors
    CHECK_WRITE(raw, 2);
    // check respose
    switch (raw->at(0)) {
        case 0: break; // no errors
        case 1: return I2CERR;
        default: return UNKNOWNERR;
    }

    // loop until self test is done
    for (int i = 0; i < 10; i++) {
        pros::delay(5); // short delay between reads
        res.start = 1;
        auto raw = m_coprocessor->write_and_receive(topic::SELF_TEST_READ, {res.start}, READ_TIMEOUT);
        // check IO errors
        CHECK_WRITE(raw, 1);
        res.value = raw->at(0);
        // check response
        if (res.value == std::numeric_limits<uint8_t>::max()) return I2CERR;
        if (!res.inProgress) break;
    }

    // check if the self test passed
    return static_cast<bool>(res.pass == 1);
}

std::expected<Status, _Error> OTOS::get_status() {
    // check that the OTOS has been initialized
    CHECK_INITIALIZE();

    // the info we receive uses bit fields
    union {
            struct {
                    uint8_t warn_tilt_angle : 1;
                    uint8_t warn_optical_tracking : 1;
                    uint8_t reserved : 4;
                    uint8_t fatal_error_optical : 1;
                    uint8_t fatal_error_imu : 1;
            };

            uint8_t value;
    } s;

    // get raw data from OTOS, and check for errors
    auto raw = m_coprocessor->write_and_receive(topic::GET_STATUS, {}, READ_TIMEOUT);
    CHECK_WRITE(raw, 1);
    // parse raw data
    s.value = raw->at(0);
    if (s.value == std::numeric_limits<uint8_t>::max()) return I2CERR;
    // return data
    return Status {
        .warn_tilt_angle = static_cast<bool>(s.warn_tilt_angle),
        .warn_optical_tracking = static_cast<bool>(s.warn_optical_tracking),
        .fatal_error_optical = static_cast<bool>(s.fatal_error_optical),
        .fatal_error_imu = static_cast<bool>(s.fatal_error_imu),
    };
}

} // namespace copro