#pragma once

#include <format>
#include <vector>
#include <ostream>

namespace copro {

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

        template <typename... Args> Error(const Error<T>& other, std::format_string<Args...> fmt, Args&&... args)
            : type(type) {
            for (auto s : other.what) what.push_back(s);
            what.push_back(std::format(fmt, std::forward<Args>(args)...));
        }

        template <typename R, typename... Args>
            requires(!std::is_same_v<R, T>)
        Error(const Error<R> other, T type, std::format_string<Args...> fmt, Args&&... args)
            : type(type) {
            for (auto s : other.what) what.push_back(s);
            what.push_back(std::format(fmt, std::forward<Args>(args)...));
        }

        const T type;
        std::vector<std::string> what;
};

template <typename T> std::ostream& operator<<(std::ostream& os, const Error<T>& e) {
    for (auto it = e.what.rbegin(); it != e.what.rend(); it++) os << "  " << *it << std::endl;
    return os;
}

} // namespace copro