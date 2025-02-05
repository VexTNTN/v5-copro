#pragma once

#include "Error.hpp"
#include "copro/Coprocessor.hpp"
#include <memory>

namespace copro {
struct Pose {
        float x;
        float y;
        float h;
};

class OTOS {
    public:
        enum class ErrorType {
            ALREADY_INITIALIZED,
            RS485_IO,
            I2C_IO,
            INCORRECT_RESPONSE_SIZE,
            NO_DEVICE,
            UNKNOWN,
        };

        struct Version {
                uint8_t major;
                uint8_t minor;
        };

        struct Status {
                /**
                 * @brief Whether the tilt angle threshold has been exceeded.
                 * If true, accelerometer data is ignored.
                 */
                bool warn_tilt_angle;
                /**
                 * @brief Whether optical tracking is unreliable.
                 * If true, only IMU data is used for tracking,
                 * unless warn_tilt_angle is set.
                 */
                bool warn_optical_tracking;
                /**
                 * @brief whether the optical sensor has a fatal error
                 */
                bool fatal_error_optical;
                /**
                 * @brief whether the IMU has a fatal error
                 */
                bool fatal_error_imu;
        };
    public:
        /**
         * @brief Construct a new OTOS
         *
         * A coprocessor is necessary to facilitate communication
         * between an Sparkfun OTOS and a VEX V5 Brain.
         * This is because an OTOS uses i2c, while a V5 Brain uses
         * RS485.
         *
         * @param coprocessor shared pointer to the coprocessor to use
         * @param device the file representing the device on the coprocessor
         */
        OTOS(std::shared_ptr<copro::Coprocessor> coprocessor, const std::string& device);
        /**
         * @brief Initialize i2c comms between the coprocessor and OTOS
         *
         * @return void on success
         * @return Error<ErrorType> on failure
         */
        std::expected<void, Error<ErrorType>> initialize();
        /**
         * @brief Check whether the OTOS is connected
         *
         * @return void on success
         * @return ErrorType<Error> on failure
         */
        std::expected<void, Error<ErrorType>> is_connected();
        /**
         * @brief Get the hardware version of the OTOS
         *
         * @return Version the hardware version
         * @return Error<ErrorType> on failure
         */
        std::expected<Version, Error<ErrorType>> get_hardware_version();
        /**
         * @brief Get the firmware version of the OTOS
         *
         * @return Version the firmware version
         * @return ErrorType on failure
         */
        std::expected<Version, Error<ErrorType>> get_firmware_version();
        /**
         * @brief Perform a self test on the OTOS
         *
         * @return bool the result of the self test
         * @return Error<ErrorType> on failure
         */
        std::expected<bool, Error<ErrorType>> self_test();
        /**
         * @brief Calibrate the IMU on the OTOS
         *
         * Calibrating the IMU removes the accelerometer and gyroscope offsets.
         *
         * @param samples Number of samples to take for calibration. Each sample takes about 2.4ms.
         * Fewer samples means faster calibration, but reduced accuracy. 255 by default.
         * @param blocking Whether the function should block until calibration has been completed.
         * true by default.
         *
         * @return void on success
         * @return Error<ErrorType> on failure
         */
        std::expected<void, Error<ErrorType>> calibrate_imu(uint8_t samples = 255, bool blocking = true);
        /**
         * @brief Whether the IMU is calibrated or not
         *
         * @return bool whether the IMU is calibrated or not
         * @return Error<ErrorType> on failure
         */
        std::expected<bool, Error<ErrorType>> is_imu_calibrated();
        /**
         * @brief Get the linear scalar used by the OTOS
         *
         * @return float the linear scalar
         * @return Error<ErrorType> on failure
         */
        std::expected<float, Error<ErrorType>> get_linear_scalar();
        /**
         * @brief Set the linear scalar used by the OTOS
         *
         * Can be used to compensate for scaling issues with the sensor measurements.
         *
         * @param scalar linear scalar, must be a value between 0.872 and 1.127
         *
         * @return void on success
         * @return Error<ErrorType> on failure
         */
        std::expected<void, Error<ErrorType>> set_linear_scalar(float scalar);
        /**
         * @brief Get the angular scalar used by the OTOS
         *
         * @return float the angular scalar
         * @return Error<ErrorType> on failure
         */
        std::expected<float, Error<ErrorType>> get_angular_scalar();
        /**
         * @brief Set the angular scalar used by the OTOS
         *
         * Can be used to compensate for scaling issues with the sensor measurements.
         *
         * @param scalar angular scalar, must be a value between 0.872 and 1.127
         *
         * @std::expected<void, Error<ErrorType>> initialize()return void on success
         * @return Error<ErrorType> on failure
         */
        std::expected<void, Error<ErrorType>> set_angular_scalar(float scalar);
        /**
         * @brief Get the status of the OTOS, which includes warning and errors reported by the sensor
         *
         * @return Status the status
         * @return Error<ErrorType> on failure
         */
        std::expected<Status, Error<ErrorType>> get_status();
        /**
         * @brief Get the offset of the OTOS
         *
         * @return Pose the offset
         * @return Error<ErrorType> on failure
         */
        std::expected<Pose, Error<ErrorType>> get_offset();
        /**
         * @brief Set the offset of the OTOS
         *
         * @param pose the offset
         *
         * @return void on success
         * @return Error<ErrorType> on failure
         */
        std::expected<void, Error<ErrorType>> set_offset(const Pose& pose);
        /**
         * @brief Get the pose measured by the OTOS
         *
         * @return Pose the pose
         * @return Error<ErrorType>> on failure
         */
        std::expected<Pose, Error<ErrorType>> get_pose();
        /**
         * @brief Set the pose of the OTOS
         *
         * @param pose the new pose
         *
         * @return void on success
         * @return Error<ErrorType>> on failure
         */
        std::expected<void, Error<ErrorType>> set_pose(const Pose& pose);
        /**
         * @brief Get the velocity measured by the OTOS
         *
         * @return Pose the velocity
         * @return Error<ErrorType>> on failure
         */
        std::expected<Pose, Error<ErrorType>> get_velocity();
        /**
         * @brief Get the acceleration measured by the OTOS
         *
         * @return Pose the acceleration
         * @return Error<ErrorType>> on failure
         */
        std::expected<Pose, Error<ErrorType>> get_acceleration();
    private:
        mutable std::shared_ptr<copro::Coprocessor> m_coprocessor;
        const std::string m_device;
        bool m_initialized = false;
};
} // namespace copro