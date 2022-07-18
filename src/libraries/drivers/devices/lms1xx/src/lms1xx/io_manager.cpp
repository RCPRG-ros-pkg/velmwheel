/* ============================================================================================================================ *//**
 * @file       io_manager.cpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Monday, 4th July 2022 3:58:11 pm
 * @modified   Thursday, 7th July 2022 3:59:17 pm
 * @project    engineering-thesis
 * @brief      Implementation of methods of the IOManager class
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

/* =========================================================== Includes =========================================================== */

// Private includes
#include "sick/lms1xx/io_manager.hpp"

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

/* ======================================================== Public methods ======================================================== */


void IOManager::write() {

    std::string_view context = (log_context != "") ? log_context : "lms1xx::IOManager::write";

    // If no socket configured, return
    if(not socket_fd.has_value())
        return;

    LOG_DEBUG("Writting command [<STX>" << get_printable_request() << "<ETX>]");

    // Write command to the device
    ::write(*socket_fd, buff, buff_size);
}


void IOManager::read() {

    // If no socket configured, return
    if(not socket_fd.has_value())
        return;

    std::string_view context = (log_context != "") ? log_context : "lms1xx::IOManager::read";
        
    // Read response
    int response_len = ::read(*socket_fd, buff, BUFF_SIZE);

    // Verify response format
    if (buff[0] != STX_MARKER)
        LOG_WARNING("Invalid packet recieved");
    if (buff[response_len - 1] != ETX_MARKER)
        LOG_WARNING("Invalid packet recieved");
        
    // Zero-terminate the response
    buff[response_len - 1] = 0;

    LOG_DEBUG("Received response [" << get_printable_response() << "]");

}


void IOManager::exchange() {

    // If no socket configured, return
    if(not socket_fd.has_value())
        return;

    std::string_view context = (log_context != "") ? log_context : "lms1xx::IOManager::exchange_packet";

    LOG_DEBUG("Writting command [<STX>" << get_printable_request() << "<ETX>]");

    // Write command to the device
    ::write(*socket_fd, buff, buff_size);
    // Read response
    int response_len = ::read(*socket_fd, buff, BUFF_SIZE);

    // Verify response format
    if (buff[0] != STX_MARKER)
        LOG_WARNING("Invalid packet recieved");
    if (buff[response_len - 1] != ETX_MARKER)
        LOG_WARNING("Invalid packet recieved");
        
    // Zero-terminate the response
    buff[response_len - 1] = 0;

    LOG_DEBUG("Received response [<STX>" << get_printable_response() << "<ETX>]");

}


int IOManager::select(timeval &timeout) {

    // If no socket configured, return
    if(not socket_fd.has_value())
        return -1;

    fd_set readset;

    // Reset FD set
    FD_ZERO(&readset);
    FD_SET(*socket_fd, &readset);

    // Wait for I/O
    return ::select(*socket_fd + 1, &readset, NULL, NULL, &timeout);
}

/* ================================================================================================================================ */

} // End namespace sick::lms1xx

/* ================================================================================================================================ */
