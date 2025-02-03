#pragma once

#include "pros/rtos.hpp"
#include <cstdint>
#include <vector>
#include <limits>

namespace copro {

/**
 * @brief Coprocessor class. Simplifies interactions with a coprocessor over a smart port
 *
 */
class Coprocessor {
    public:
        Coprocessor(int port, int baud_rate);
        std::vector<uint8_t> write_and_receive(uint8_t id, const std::vector<uint8_t>& data, int timeout);
        /**
         * @brief initialize comms with the coprocessor.
         *
         * This function will block until the coprocessor responds to a ping,
         * or until one of the lambda functions return true.
         *
         * @param timeout max time that can be taken to initialize, in milliseconds. integer max by default
         */
        int initialize(int timeout = std::numeric_limits<int>::max());
    private:
        const int baud;
        const int m_port;
        pros::Mutex m_mutex;
};
} // namespace copro