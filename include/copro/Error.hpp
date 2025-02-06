#pragma once

#include <format>
#include <vector>
#include <ostream>
#include <expected>

namespace copro {

template <typename T>
    requires std::is_scoped_enum_v<T>
class Error {
    public:
        /**
         * @brief create a new error
         *
         * @tparam Args std::format args types
         *
         * @param type the type of error
         * @param fmt formatter string
         * @param args formatter arguments
         *
         * @return std::unexpected<Error>
         */
        template <typename... Args>
        static constexpr std::unexpected<Error<T>> make(T type, std::format_string<Args...> fmt, Args&&... args) {
            return std::unexpected<Error<T>>(Error<T>(type, {std::format(fmt, std::forward<Args>(args)...)}));
        }

        template <typename R, typename... Args> static constexpr std::unexpected<Error<T>>
        add(const std::expected<R, Error<T>>& other, std::format_string<Args...> fmt, Args&&... args) {
            auto tmp = other.error();
            tmp.what.push_back(std::format(fmt, std::forward<Args>(args)...));
            return std::unexpected<Error<T>>(tmp);
        }

        template <typename R, typename... Args> static constexpr std::unexpected<Error<T>>
        add(std::expected<R, Error<T>>&& other, std::format_string<Args...> fmt, Args&&... args) {
            other.what.push_back(std::format(fmt, std::forward<Args>(args)...));
            return std::unexpected<Error<T>>(std::move(other));
        }

        template <typename R, typename S, typename... Args>
            requires(!std::is_same_v<T, R>)
        static constexpr std::unexpected<Error<T>> add(const std::expected<R, Error<S>>& other, T type,
                                                       std::format_string<Args...> fmt, Args&&... args) {
            std::vector<std::string> tmp = other.error().what;
            tmp.push_back(std::format(fmt, std::forward<Args>(args)...));
            return std::unexpected<Error<T>>(Error<T>(type, std::move(tmp)));
        }

        template <typename R, typename S, typename... Args>
            requires(!std::is_same_v<T, R>)
        static constexpr std::unexpected<Error<T>> add(std::expected<R, Error<S>>&& other, T type,
                                                       std::format_string<Args...> fmt, Args&&... args) {
            other.what.push_back(std::format(fmt, std::forward<Args>(args)...));
            return std::unexpected<Error<T>>(Error<T>(type, std::move(other)));
        }

        const T type;
        std::vector<std::string> what;
    private:
        Error(T type, std::vector<std::string>&& s)
            : type(type),
              what({s}) {}
};

template <typename T> std::ostream& operator<<(std::ostream& os, const Error<T>& e) {
    for (auto it = e.what.rbegin(); it != e.what.rend(); it++) os << "  " << *it << std::endl;
    return os;
}

} // namespace copro