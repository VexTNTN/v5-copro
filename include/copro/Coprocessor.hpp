#pragma once

#include "copro/Error.hpp"
#include "pros/rtos.hpp"
#include <cstdint>
#include <expected>
#include <vector>

namespace copro {

/**
 * @brief Coprocessor class. Simplifies interactions with a coprocessor over a smart port
 *
 */
class Coprocessor {
    public:
        enum class ErrorType {

        };
    public:
        Coprocessor(int port, int baud_rate);
        /**
         * @brief
         *
         * @return std::unexpected<Error<ErrorType>>
         */
        std::unexpected<Error<ErrorType>> initialize();
        std::expected<std::vector<uint8_t>, Error<ErrorType>>
        write_and_receive(const std::string& topic, const std::vector<uint8_t>& data, int timeout);
    private:
        std::expected<int, Error<ErrorType>> find_id(const std::string& topic);

        const int m_port;
        const int m_baud_rate;
        bool m_initialized = false;
        pros::Mutex m_mutex;
        std::vector<std::string> m_topics = {"register", "ping", "version"};
};
} // namespace copro