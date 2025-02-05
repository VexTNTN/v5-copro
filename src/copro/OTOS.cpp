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

#define ERROR(...) std::unexpected<_Error>(_Error(__VA_ARGS__))

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
constexpr const char* IS_CONNECTED = "otos/is_connected";
constexpr const char* GET_HARDWARE_VERSION = "otos/get_hardware_version";
constexpr const char* GET_FIRMWARE_VERSION = "otos/get_firmware_version";
constexpr const char* SELF_TEST = "otos/self_test";
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
    // check if the OTOS has already been initialized
    if (m_initialized) {
        return ERROR(ALREADY_INITIALIZED, "OTOS with id {} on port {} has already been initialized", m_device,
                     m_coprocessor->get_port());
    }
    // prepare data
    std::vector<uint8_t> out;
    for (auto c : m_device) out.push_back(c);
    // send data
    auto raw = m_coprocessor->write_and_receive(topic::INITIALIZE, out, READ_TIMEOUT);
    // check for errors
    if (!raw) {
        return std::unexpected(_Error(raw.error(), RS485_IO, "failed to send command to OTOS with id {} on port {}",
                                      m_device, m_coprocessor->get_port()));
    }
    if (raw->size() != 1) {
        return ERROR(INCORRECT_RESPONSE_SIZE, "incorrect response size from OTOS with id {} on port {}", m_device,
                     m_coprocessor->get_port());
    }
    if (raw->at(0) == 1) {
        // the OTOS was initialized before this program was ran, probably
        // not a big deal, so we eat the "error"
        m_initialized = true;
        return {};
    }
    if (raw->at(0) == 2) {
        return ERROR(I2C_IO, "failed to interact with OTOS with id {} on port {}", m_device, m_coprocessor->get_port());
    }
    if (raw->at(0) != 0) {
        return ERROR(UNKNOWN, "unknown error response when trying to interact with OTOS with id {} on port {}",
                     m_device, m_coprocessor->get_port());
    }
    // no errors
    return {};
}

std::expected<Status, _Error> OTOS::get_status() {
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
    if (!raw) {
        return std::unexpected(_Error(raw.error(), RS485_IO, "failed to send command to OTOS with id {} on port {}",
                                      m_device, m_coprocessor->get_port()));
    }
    if (raw->size() != 2) return ERROR(INCORRECT_RESPONSE_SIZE, "response from coprocessor too short");
    // parse raw data
    s.value = raw->at(0);
    if (s.value == std::numeric_limits<uint8_t>::max()) {
        return ERROR(I2C_IO, "failed to interact with OTOS with id {} on port {}", m_device, m_coprocessor->get_port());
    }
    // return data
    return Status {
        .warn_tilt_angle = static_cast<bool>(s.warn_tilt_angle),
        .warn_optical_tracking = static_cast<bool>(s.warn_optical_tracking),
        .fatal_error_optical = static_cast<bool>(s.fatal_error_optical),
        .fatal_error_imu = static_cast<bool>(s.fatal_error_imu),
    };
}

} // namespace copro