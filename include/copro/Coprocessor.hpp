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

        /**
         * @brief initialize comms with the coprocessor.
         *
         * This function will block until the coprocessor responds to a ping,
         * or until one of the lambda functions return true.
         *
         * @param timeout max time that can be taken to initialize, in milliseconds. integer max by default
         */
        int initialize(int timeout = std::numeric_limits<int>::max());
        std::vector<uint8_t> write_and_receive(const std::string& topic, const std::vector<uint8_t>& data, int timeout);
    private:
        int find_id(const std::string& topic);

        const int m_port;
        const int m_baud_rate;
        pros::Mutex m_mutex;
        std::vector<std::string> m_topics = {"register", "ping", "version"};
};
} // namespace copro