#pragma once

#include <expected>
#include <format>
#include <ostream>
#include <vector>

namespace copro {

template<typename T>
    requires std::is_scoped_enum_v<T>
class Error {
    public:
        /**
         * @brief create a new error
         *
         * @tparam Args std::format arguments types
         *
         * @param type the type of error
         * @param fmt formatter string
         * @param args formatter arguments
         *
         * @return constexpr std::unexpected<Error>
         */
        template<typename... Args>
        static constexpr std::unexpected<Error<T>>
        make(T type, std::format_string<Args...> fmt, Args&&... args) noexcept {
            return std::unexpected<Error<T>>(
              Error<T>(type,
                       { std::format(fmt, std::forward<Args>(args)...) }));
        }

        /**
         * @brief add more info to an error
         *
         * @tparam R the expected value
         * @tparam Args std::format arguments types
         *
         * @param other the error to add to
         * @param fmt formatter string
         * @param args formatter arguments
         *
         * @return constexpr std::unexpected<Error<T>>
         */
        template<typename R, typename... Args>
        static constexpr std::unexpected<Error<T>>
        add(const std::expected<R, Error<T>>& other,
            std::format_string<Args...> fmt,
            Args&&... args) {
            auto tmp = other.error();
            tmp.what.push_back(std::format(fmt, std::forward<Args>(args)...));
            return std::unexpected<Error<T>>(tmp);
        }

        /**
         * @brief add more info to an error
         *
         * @tparam R the expected value
         * @tparam Args std::format arguments types
         *
         * @param other the error to add to
         * @param fmt formatter string
         * @param args formatter arguments
         *
         * @return constexpr std::unexpected<Error<T>>
         */
        template<typename R, typename... Args>
        static constexpr std::unexpected<Error<T>>
        add(std::expected<R, Error<T>>&& other,
            std::format_string<Args...> fmt,
            Args&&... args) {
            other.what.push_back(std::format(fmt, std::forward<Args>(args)...));
            return std::unexpected<Error<T>>(std::move(other));
        }

        /**
         * @brief add more info to an error
         *
         * @tparam R the expected value
         * @tparam S the type of the error to add to
         * @tparam Args std::format arguments types
         *
         * @param other the error to add to
         * @param fmt formatter string
         * @param args formatter arguments
         *
         * @return constexpr std::unexpected<Error<T>>
         */
        template<typename R, typename S, typename... Args>
            requires(!std::is_same_v<T, R>)
        static constexpr std::unexpected<Error<T>>
        add(T type,
            const std::expected<R, Error<S>>& other,
            std::format_string<Args...> fmt,
            Args&&... args) {
            std::vector<std::string> tmp = other.error().what;
            tmp.push_back(std::format(fmt, std::forward<Args>(args)...));
            return std::unexpected<Error<T>>(Error<T>(type, std::move(tmp)));
        }

        /**
         * @brief add more info to an error
         *
         * @tparam R the expected value
         * @tparam S the type of the error to add to
         * @tparam Args std::format arguments types
         *
         * @param other the error to add to
         * @param fmt formatter string
         * @param args formatter arguments
         *
         * @return constexpr std::unexpected<Error<T>>
         */
        template<typename R, typename S, typename... Args>
            requires(!std::is_same_v<T, R>)
        static constexpr std::unexpected<Error<T>>
        add(T type,
            std::expected<R, Error<S>>&& other,
            std::format_string<Args...> fmt,
            Args&&... args) {
            other.what.push_back(std::format(fmt, std::forward<Args>(args)...));
            return std::unexpected<Error<T>>(Error<T>(type, std::move(other)));
        }

        /**
         * @brief the type of error
         */
        const T type;
        /**
         * @brief information of the errors
         */
        std::vector<std::string> what;

    private:
        Error(T type, std::vector<std::string>&& s) noexcept
            : type(type),
              what({ s }) {}
};

template<typename T>
std::ostream& operator<<(std::ostream& os, const Error<T>& e) {
    for (int i = e.what.size() - 1; i >= 0; i--) {
        for (int j = 0; j < e.what.size() - 1 - i; j++) os << "  ";
        os << e.what.at(i) << std::endl;
    }
    return os;
}

} // namespace copro
