/* ============================================================================================================================ *//**
 * @file       io_manager.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Monday, 4th July 2022 3:06:48 pm
 * @modified   Thursday, 7th July 2022 3:58:58 pm
 * @project    engineering-thesis
 * @brief      Implementation of the inline methods of the IOManager class
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __SICK_LMS1XX_IO_MANAGER_IO_MANAGER_H__
#define __SICK_LMS1XX_IO_MANAGER_IO_MANAGER_H__

/* =========================================================== Includes =========================================================== */

// Standard includes
#include <cstring>
// Linux includes
#include <unistd.h>
// Private includes
#include "sick/lms1xx/io_manager.hpp"
#include "sick/lms1xx/format.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace sick::lms1xx {

/* ======================================================= Auxiliary macros ======================================================= */

/// Logs debug message in a stream-like fashion, if logger is configrued
#define LOG_DEBUG(...)    \
    if(logger != nullptr) \
        RCLCPP_DEBUG_STREAM(*logger, "[" << io_manager::context_base << "::" << context << "] " << __VA_ARGS__)

/// Logs warning message in a stream-like fashion, if logger is configrued
#define LOG_WARNING(...)    \
    if(logger != nullptr) \
        RCLCPP_WARN_STREAM(*logger, "[" << io_manager::context_base << "::" << context << "] " << __VA_ARGS__)

/* ====================================================== Auxiliary constants ===================================================== */

namespace io_manager {

    /// Context string for log messages
    static constexpr auto context_base = "sick";

}

/* ========================================================= Public ctors ========================================================= */

IOManager::IOManager(
    std::optional<int> socket_fd,
    rclcpp::Logger *logger,
    std::string_view log_context
) :
    logger{ logger },
    log_context{ log_context},
    socket_fd{ socket_fd }
{ }

/* ======================================================== Public methods ======================================================== */

void IOManager::format_cmd_impl(const char *fmt, std::va_list *args) {
     
    // Format command header
    snprintf (buff, BUFF_SIZE, "%c", STX_MARKER);
    // Format command body
    vsnprintf(buff + 1, BUFF_SIZE - 1, fmt, *args);
    // Format command footer
    buff_size = strlen(buff);
    snprintf (buff + buff_size, BUFF_SIZE - buff_size, "%c", ETX_MARKER);
    
    // Update buff size
    ++buff_size;
}


void IOManager::format_cmd(const char *fmt, ...) {

    std::va_list args;

    // Parse arguments
    va_start(args, fmt);
    
    // Format command
    format_cmd_impl(fmt, &args);

    // Deinitialize va_list
    va_end(args);

}


void IOManager::write(const char *fmt, ...) {

    std::va_list args;

    // Parse arguments
    va_start(args, fmt);
    
    // Format command
    format_cmd_impl(fmt, &args);
    // Exchange data
    write();

    // Deinitialize va_list
    va_end(args);
}


void IOManager::exchange(const char *fmt, ...) {

    std::va_list args;

    // Parse arguments
    va_start(args, fmt);
    
    // Format command
    format_cmd_impl(fmt, &args);
    // Exchange data
    exchange();

    // Deinitialize va_list
    va_end(args);
}


std::string_view IOManager::get_request() const {
    return buff;
}


std::string IOManager::get_printable_request() const {

    std::string_view context = (log_context != "") ? log_context : "lms1xx::IOManager::get_printable_request";

    // Get request string
    auto request = get_request();
    // Find position of the <ETX> marker
    auto etx_position = std::find(request.begin(), request.end(), static_cast<char>(ETX_MARKER));

    // Verify whether <ETX> is present (should be always)
    if(etx_position == request.end())
        LOG_WARNING("Request buffer does NOT contain <ETX> marker");

    return std::string{ request.data() + 1, etx_position };
}


std::string_view IOManager::get_response() const {
    return &(buff[1]);
}


std::string IOManager::get_printable_response() const {
    return std::string{ get_response() };
}

/* ============================================================ Cleanup =========================================================== */

#undef LOG_DEBUG
#undef LOG_WARNING

/* ================================================================================================================================ */

} // End namespace sick::lms1xx

/* ================================================================================================================================ */

#endif
