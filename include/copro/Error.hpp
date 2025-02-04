#pragma once

#include <format>
#include <sstream>
#include <string>
#include <source_location>
#include <vector>

namespace copro {

/**
 * @brief Get the name of enum values
 *
 * This function needs to be specialized to work with a given type
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
class Error {
    public:
        template <typename... Args> Error(T type, std::format_string<Args...> fmt, Args&&... args)
            : type(type),
              what({std::format(fmt, std::forward<Args>(args)...)}) {}

        template <typename... Args> Error(const Error& other, std::format_string<Args...> fmt, Args&&... args)
            : type(type) {
            for (auto s : other.what) what.push_back(s);
            what.push_back(std::format(fmt, std::forward<Args>(args)...));
        }

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
            for (auto s : what) out << "  " << s << std::endl;
            return out.str();
        }

        const T type;
        std::vector<std::string> what;
};

} // namespace copro