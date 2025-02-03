#pragma once

#include "pros/rtos.hpp"
#include <cstdint>
#include <vector>
#include <limits>
#include <algorithm>

namespace copro {

// TODO: make a struct with the serialization/deserialization functions instead
// but still have it be compile-time polymorphism.

/**
 * @brief serialize data
 *
 * This function serializes any data that is trivially copyable.
 * For more complex data types, it can be specialized.
 *
 * @tparam T the type of the data to serialize
 * @param data the data to serialize
 * @return std::vector<uint8_t> the serialized data
 */
template <typename T> std::vector<uint8_t> serialize(const T& data) {
    static_assert(std::is_trivially_copyable_v<T>, "Type must be trivially copyable");
    auto raw = std::bit_cast<std::array<uint8_t, sizeof(T)>>(data);
    std::vector<uint8_t> out(sizeof(T));
    for (int i = 0; i < sizeof(T); i++) out.at(i) = raw.at(i);
    return out;
}

/**
 * @brief deserialize data
 *
 * This function deserializes any data that is trivially copyable.
 * For more complex data types, it can be specialized.
 *
 * @tparam T the type that the data should be deserialized to
 * @param data the data to deserialize
 * @return T deserialized output
 */
template <typename T> T deserialize(const std::vector<uint8_t>& data) {
    static_assert(std::is_trivially_copyable_v<T>, "Type must be trivially copyable");
    std::array<uint8_t, sizeof(T)> raw;
    for (int i = 0; i < sizeof(T); i++) raw.at(i) = data.at(i);
    return std::bit_cast<T>(raw);
}

/**
 * Literal class type that wraps a constant expression string.
 *
 * Uses implicit conversion to allow templates to *seemingly* accept constant strings.
 */
template <size_t N> struct StringLiteral {
        constexpr StringLiteral(const char (&str)[N]) { std::copy_n(str, N, value); }

        char value[N];
};

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

        /**
         * @brief send data of an arbitrary type, and receive data of an arbitrary type
         *
         * @tparam topic
         * @tparam T
         * @tparam R
         * @param data
         * @param timeout
         * @return T
         */
        template <StringLiteral topic, typename T, typename R> R write_and_receive(T data, int timeout) {
            return deserialize<R>(write_and_receive(topic, serialize<T>(data), timeout));
        }
    private:
        const int baud_rate;
        const int m_port;
        pros::Mutex m_mutex;
        std::vector<std::string> m_topics;
};
} // namespace copro