/* ============================================================================================================================ *//**
 * @file       io_manager.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Monday, 4th July 2022 3:06:48 pm
 * @modified   Thursday, 7th July 2022 2:36:18 pm
 * @project    engineering-thesis
 * @brief      Auxiliary buffer structure automating UDP data parsing for LMS1xx driver class
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __SICK_LMS1XX_IO_MANAGER_H__
#define __SICK_LMS1XX_IO_MANAGER_H__

/* =========================================================== Includes =========================================================== */

// Standard includes
#include <cstdint>
#include <cstdarg>
#include <string_view>
// ROS includes
#include "rclcpp/rclcpp.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace sick::lms1xx {
    
/* ============================================================= Class ============================================================ */

/**
 * @brief Auxiliary class providing unified interface for exhcnaging commands
 *    with the target LIDAR 
 */
class IOManager {
public: /* Public constants */

    /// Size of the underlying I/O buffer (can be modified to optimize performance)
    static constexpr std::size_t BUFF_SIZE = 100;

public: /* Public methods */

    /**
     * @brief Constructs a new IOManager object
     * 
     * @param socket_fd 
     *    file descriptor of the UDP socket
     * @param logger 
     *    (optional) logger interface to be used
     * @param log_context 
     *    (optional) logger context string to be used
     */
    inline IOManager(
        std::optional<int> socket_fd,
        rclcpp::Logger *logger = nullptr,
        std::string_view log_context = ""
    );

    /**
     * @brief Formats command to be sent on the next @ref exchange() call
     * 
     * @param fmt 
     *    format of the command
     * @param ... 
     *    format-dependent arguments
     */
    inline void format_cmd(const char *fmt, ...);

    /**
     * @brief Writes currently formated command to the device
     */
    void write();

    /**
     * @brief Alias for format_cmd(fmt, ...); write();
     */
    inline void write(const char *fmt, ...);

    /**
     * @brief Reads response from the device
     */
    void read();

    /**
     * @brief Exchanges currently formatted command with the LIDAR
     * 
     * @returns 
     *    \0-terminated response
     */
    void exchange();

    /**
     * @brief Alias for format_cmd(fmt, ...); exchange();
     */
    inline void exchange(const char *fmt, ...);

    /**
     * @brief Waits for response from the device using select()
     * 
     * @param timeout 
     *    select() timeout
     * 
     * @returns 
     *    result of the select() call
     */
    int select(timeval &timeout);

    /**
     * @brief Get the request buffer
     * 
     * @returns 
     *    response received with the last @a exhcnage call
     * 
     * @note Request buffer returned by the method DOES contain <STX> and <ETX> markers
     */
    inline std::string_view get_request() const;

    /**
     * @brief Get the request buffer translated to null-terminated string
     * 
     * @returns 
     *    response received with the last @a exhcnage call
     * 
     * @note Request buffer returned by the method DOES contain <STX> and <ETX> markers
     */
    inline std::string get_printable_request() const;

    /**
     * @brief Get the response buffer
     * 
     * @returns 
     *    response received with the last @a exhcnage call
     * 
     * @note Response buffer returned by the method does NOT contain <STX> and <ETX> markers. They are remove from the
     *    response when it is received with any of I/O methods. Returned string is null-terminater.
     */
    inline std::string_view get_response() const;

    /**
     * @brief Get the response buffer translated to null-terminated string
     * 
     * @returns 
     *    response received with the last @a exhcnage call
     * 
     * @note Response buffer returned by the method does NOT contain <STX> and <ETX> markers. They are remove from the
     *    response when it is received with any of I/O methods. Returned string is null-terminater.
     */
    inline std::string get_printable_response() const;

private: /* Private methods */

    /**
     * @brief Formats command to be sent on the next @ref exchange() call
     * 
     * @param fmt 
     *    format of the command
     * @param ... 
     *    format-dependent arguments
     */
    inline void format_cmd_impl(const char *fmt, std::va_list *args);

private: /* Private data */

    /// Logger interface (optional)
    rclcpp::Logger *logger { nullptr };
    /// Custom log context string (optional)
    std::string_view log_context = "";
    
    /// Socket descriptor for UDP connection (no value when driver not connected to the LIDAR)
    std::optional<int> socket_fd;

    /// Command/response buffer size
    char buff[BUFF_SIZE] { "" };
    /// Number of bytes in buffer that are used
    std::size_t buff_size { 0 };

};

/* ================================================================================================================================ */

} // End namespace sick::lms1xx

/* ==================================================== Implementation includes =================================================== */

#include "sick/lms1xx/io_manager/io_manager.hpp"

/* ================================================================================================================================ */

#endif
