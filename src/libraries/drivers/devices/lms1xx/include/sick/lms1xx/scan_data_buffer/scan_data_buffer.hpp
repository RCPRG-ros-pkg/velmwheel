/* ============================================================================================================================ *//**
 * @file       scan_data_buffer.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Monday, 4th July 2022 3:06:48 pm
 * @modified   Thursday, 7th July 2022 3:58:46 pm
 * @project    engineering-thesis
 * @brief      Implementation of the auxiliary buffer structure automating UDP data parsing for LMS1xx driver class
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __SICK_LMS1XX_SCAN_DATA_BUFFER_SCAN_DATA_BUFFER_H__
#define __SICK_LMS1XX_SCAN_DATA_BUFFER_SCAN_DATA_BUFFER_H__

/* =========================================================== Includes =========================================================== */

// Standard includes
#include <cstring>
// Linux includes
#include <unistd.h>
// Private includes
#include "sick/lms1xx/scan_data_buffer.hpp"
#include "sick/lms1xx/format.hpp"

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

/* ====================================================== Auxiliary constants ===================================================== */

namespace scan_data_buffer {

    /// Context string for log messages
    static constexpr auto context_base = "sick::lms1xx::ScanDataBuffer";

}

/* ========================================================= Public ctors ========================================================= */

ScanDataBuffer::ScanDataBuffer(rclcpp::Logger &logger) :
    logger{ &logger }
{ }

/* ======================================================== Public methods ======================================================== */

void ScanDataBuffer::read(int fd) {

    static constexpr auto context = "read";

    // Log debugging info
    LOG_DEBUG("Reading bytes from socket (current buffer size is " << buffer_size << ")");

    // Read data info the buffer
    int ret = ::read(
        fd,
        buffer + buffer_size,
        sizeof(buffer) - buffer_size
    );

    // If I/O succeeded, update buffer length
    if(ret > 0) {

        // Update buffer length
        buffer_size += static_cast<std::size_t>(ret);

        // Log debugging info
        LOG_DEBUG("Read (" << ret << ") bytes from fd (current buffer length is " << buffer_size << ")");

    // Otherwise, print warning message
    } else 
        LOG_WARNING("ScanDataBuffer read() returned error.");

}


void ScanDataBuffer::pop_last_buffer() {

    // If end of the first message has been found
    if(first_message_end != nullptr) {

        // Erase first message from the buffer
        shift_buffer(first_message_end + 1);

        // Reset pointer to the message end
        first_message_end = NULL;
    }
    
}


void ScanDataBuffer::shift_buffer(uint8_t *new_start) {

    // Calculate remaining size of the buffer after shift
    int remaining_size = (buffer_size - (new_start - buffer));

    // If valid @p new_start given, shift the buffer
    if (remaining_size > 0)
        memmove(buffer, new_start, remaining_size);
    
    // Update buffer's length
    buffer_size = remaining_size;

    /**
     * @note This method is quite unsafe as it moves content only if @p new_start is IN RANGE
     *    of the current buffer, but updates it's length nevertheless. This has not been changed
     *    when porting the driver due to possible unknown side effects.
     */

}

/* ============================================================ Cleanup =========================================================== */

#undef LOG_DEBUG
#undef LOG_WARNING

/* ================================================================================================================================ */

} // End namespace sick::lms1xx

/* ================================================================================================================================ */

#endif
