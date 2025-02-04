#pragma once

#include "copro/Coprocessor.hpp"
#include <memory>

namespace copro {
struct Pose {
        float x;
        float y;
        float h;
};

struct Status {
        bool warn_tilt_angle;
        bool warn_optical_tracking;
        bool optical_fatal;
        bool imu_fatal;
        bool pros_error;
};

struct Version {
        uint8_t major;
        uint8_t minor;
};

class OTOS {
    public:
        OTOS(std::shared_ptr<copro::Coprocessor> coprocessor);
        Status get_status();
        Version get_version();
        int self_test();

        int calibrate(uint8_t samples = 255);
        int is_calibrated();
        int reset_tracking();

        Pose get_offset();
        int set_offset(Pose pose);
        float get_linear_scalar();
        int set_linear_scalar(float scalar);
        float get_angular_scalar();
        int set_angular_scalar(float scalar);

        Pose get_pose();
        int set_pose(Pose pose);
    private:
        std::shared_ptr<copro::Coprocessor> m_coprocessor;
};
} // namespace copro