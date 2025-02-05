#pragma once

#include "copro/Error.hpp"
#include "pros/rtos.hpp"
#include <cstdint>
#include <expected>
#include <vector>

namespace copro {

// protocol:
// it's all COBS'd
// [8-bit ID] [CRC16] [payload] [0x0]

/**
 * @brief Coprocessor class. Simplifies interactions with a coprocessor over a smart port
 *
 */
class Coprocessor {
    public:
        enum class ErrorType {
            READ_TIMEOUT,
            INVALID_PORT,
            PORT_UNAVAILABLE,
            NO_MESSAGE,
            MESSAGE_CORRUPTED,
            MESSAGE_TOO_BIG,
            IO_FAILURE,
            NOT_INITIALIZED,
            ALREADY_INITIALIZED,
            UNKNOWN_FAILURE,
            UNKNOWN_FAILURE_COPROCESSOR,
            TOO_MANY_TOPICS,
        };
    public:
        Coprocessor(int port, int baud_rate);
        /**
         * @brief
         *
         * @return std::unexpected<Error<ErrorType>>
         */
        std::expected<int, Error<ErrorType>> initialize();
        std::expected<std::vector<uint8_t>, Error<ErrorType>>
        write_and_receive(const std::string& topic, const std::vector<uint8_t>& data, int timeout);
        int get_port();
    private:
        std::expected<uint8_t, Error<ErrorType>> find_id(const std::string& topic);

        const int m_port;
        const int m_baud_rate;
        bool m_initialized = false;
        pros::Mutex m_mutex;
        std::vector<std::string> m_topics = {"register", "ping", "version"};
};
} // namespace copro