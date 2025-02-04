#pragma once

#include <sstream>
#include <string>
#include <source_location>

namespace copro {

/**
 * @brief Get the name of enum values
 *
 * @tparam T the enum
 * @param err the value
 * @return std::string the name of the enum value as a string
 */
template <typename T>
    requires std::is_scoped_enum_v<T>
std::string get_error_name(T err);

/**
 * @brief Generic Error class
 *
 * @tparam T the error enum to use
 */
template <typename T>
    requires std::is_scoped_enum_v<T>
struct Error {
        T type;
        std::string what;

        /**
         * @brief get the error as a string
         *
         * @return std::string
         */
        std::string as_string(std::source_location loc = std::source_location::current()) {
            std::ostringstream out;
            out << get_error_name<T>(type) << " Error at:" << std::endl;
            out << "  " << loc.file_name() << '(' << loc.line() << ':' << loc.column() << ")" << std::endl;
            out << "in function:" << std::endl;
            out << "  " << loc.function_name() << std::endl;
            out << "with message:" << std::endl;
            out << "  " << what << std::endl;
            return out.str();
        }
};

} // namespace copro