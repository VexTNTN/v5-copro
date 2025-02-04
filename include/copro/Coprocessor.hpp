#pragma once

#include "pros/rtos.hpp"
#include <cstdint>
#include <vector>

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
         * This function will block until the coprocessor responds to a ping.
         *
         * This function may set errno to one of the following values upon failure:
         *
         *
         * @param timeout max time that can be taken to initialize, in milliseconds. integer max by default
         *
         * @return 0 on success
         * @return INT32_MAX on failure, setting errno
         */
        int initialize();
        std::vector<uint8_t> write_and_receive(const std::string& topic, const std::vector<uint8_t>& data, int timeout);
    private:
        int find_id(const std::string& topic);

        const int m_port;
        const int m_baud_rate;
        bool m_initialized = false;
        pros::Mutex m_mutex;
        std::vector<std::string> m_topics = {"register", "ping", "version"};
};
} // namespace copro