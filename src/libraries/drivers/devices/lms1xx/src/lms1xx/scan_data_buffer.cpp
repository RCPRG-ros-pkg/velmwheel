/* ============================================================================================================================ *//**
 * @file       scan_data_buffer.cpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Monday, 4th July 2022 3:06:48 pm
 * @modified   Thursday, 7th July 2022 4:31:35 pm
 * @project    engineering-thesis
 * @brief      Implementation of the auxiliary buffer structure automating UDP data parsing for LMS1xx driver class
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

/* =========================================================== Includes =========================================================== */

// Standard includes
#include <cstring>
// Private includes
#include "sick/lms1xx/scan_data_buffer.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace sick::lms1xx {

/* ======================================================= Auxiliary macros ======================================================= */

/// Logs debug message in a stream-like fashion, if logger is configrued
#define LOG_DEBUG(...)    \
    if(logger != nullptr) \
        RCLCPP_DEBUG_STREAM(*logger, "[" << scan_data_buffer::context_base << "::" << context << "] " << __VA_ARGS__)

/// Logs warning message in a stream-like fashion, if logger is configrued
#define LOG_WARNING(...)    \
    if(logger != nullptr) \
        RCLCPP_WARN_STREAM(*logger, "[" << scan_data_buffer::context_base << "::" << context << "] " << __VA_ARGS__)

/* ======================================================== Public methods ======================================================== */

uint8_t* ScanDataBuffer::get_next_message() {

    static constexpr auto context = "get_next_message";

    // If buffer is empty, no scan data present
    if (buffer_size == 0) {
        LOG_DEBUG("Empty buffer, nothing to return.");
        return nullptr;
    }

    /*
     * The objective is to have a message starting at the start of the buffer, so if that's not
     * the case, then we look for a start-of-message character and shift the buffer back, discarding
     * any characters in the middle.
    */
    uint8_t *start_of_message = static_cast<uint8_t*>(std::memchr(
        static_cast<void*>(buffer),
        STX_MARKER,
        buffer_size
    ));

    // No message found - reset buffer
    if (start_of_message == nullptr) {

        LOG_WARNING("No <STX> marker found, dropping (" << buffer_size << ") bytes from buffer.");

        // Reset buffer size
        buffer_size = 0;

    // Else, if message starts at the beggining of the buffer - shift the buffer back
    } else if (buffer != start_of_message) {

        LOG_WARNING("Shifting buffer, dropping (" << (start_of_message - buffer) << ") bytes, (" << (buffer_size - (start_of_message - buffer)) << ") bytes remain.");

        // Shift the buffer
        shift_buffer(start_of_message);
    }

    // Now look for the end of message character.
    first_message_end = static_cast<uint8_t*>(std::memchr(
        static_cast<void*>(buffer),
        ETX_MARKER,
        buffer_size
    ));

    // No end of message found - no message to parse and return.
    if (first_message_end == nullptr) {

        LOG_DEBUG("No <ETX> marker found, nothing to return.");

        // Return nothing
        return nullptr;
    }

    // Null-terminate the buffer.
    *first_message_end = 0;

    LOG_DEBUG("Scan data message parsed: " << (buffer + 1));

    // Return pointer to the message start
    return (buffer + 1);
}

/* ================================================================================================================================ */

} // End namespace sick::lms1xx

/* ================================================================================================================================ */
