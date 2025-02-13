#pragma once

#include "copro/Error.hpp"
#include "pros/rtos.hpp"
#include <cstdint>
#include <expected>
#include <vector>

namespace copro {

/**
 * @brief get an object as a vector of uint8_t
 *
 * @tparam T the type of the object. Has to be trivially copyable
 *
 * @param t the object to serialize
 *
 * @return std::vector<uint8_t> the object as a vector of uint8_t
 */
template<typename T>
    requires std::is_trivially_copyable<T>::value
std::vector<uint8_t> serialize(T t) {
    const auto raw = std::bit_cast<std::array<uint8_t, sizeof(T)>>(t);
    std::vector<uint8_t> out;
    for (const uint8_t b : raw) out.push_back(b);
    return out;
}

/**
 * @brief turn a vector of uint8_t into a vector of uint8_t
 *
 * @tparam T the type of the object. Has to be trivially copyable
 *
 * @param v the vector to interpret
 *
 * @return T the deserialized object
 */
template<typename T>
    requires std::is_trivially_copyable<T>::value
T deserialize(const std::vector<uint8_t>& v) {
    std::array<uint8_t, sizeof(T)> tmp;
    for (int i = 0; i < tmp.size(); i++) {
        tmp.at(i) = v.at(i);
    }
    return std::bit_cast<T>(tmp);
}

/**
 * @brief Coprocessor class. Simplifies interactions with a coprocessor over a
 * smart port
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
    /**
     * @brief Construct a new Coprocessor object
     *
     * @param port the port the coprocessor is connected to
     * @param baud_rate the baud rate of the coprocessor. 921600 recommended
     */
    Coprocessor(int port, int baud_rate);
    /**
     * @brief Initialize a the port of the coprocessor as a generic serial port
     *
     * @note this function needs to be run before any other member functions
     *
     * @return void on success
     * @return Error<ErrorType> on failure
     */
    std::expected<void, Error<ErrorType>> initialize();
    /**
     * @brief write data to the coprocessor, and receive data from the
     * coprocessor
     *
     * @param topic the topic string
     * @param data the data to write
     * @param timeout how much time the coprocessor can take to prepare the
     * response
     *
     * @return std::vector<uint8_t> the received bytes, on success
     * @return Error<ErrorType>> on failure
     */
    std::expected<std::vector<uint8_t>, Error<ErrorType>>
    write_and_receive(const std::string& topic,
                      const std::vector<uint8_t>& data,
                      int timeout);
    /**
     * @brief Get the port of the Coprocessor
     *
     * @return int the port
     */
    int get_port() const;
    /**
     * @brief Get the baud rate of the generic serial port used by the
     * Coprocessor
     *
     * @return int the baud
     */
    int get_baud_rate() const;

  private:
    std::expected<uint8_t, Error<ErrorType>> read_byte();
    std::expected<std::vector<uint8_t>, Error<ErrorType>> read();
    std::expected<uint8_t, Error<ErrorType>> find_id(const std::string& topic);

    const int m_port;
    const int m_baud_rate;
    const double m_micros_per_byte;
    bool m_initialized = false;
    pros::Mutex m_mutex;
    std::vector<std::string> m_topics = { "register", "ping", "version" };
};
} // namespace copro
