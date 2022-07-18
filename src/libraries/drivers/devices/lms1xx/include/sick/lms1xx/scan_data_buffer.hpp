/* ============================================================================================================================ *//**
 * @file       buffer.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Monday, 4th July 2022 3:06:48 pm
 * @modified   Monday, 4th July 2022 10:43:17 pm
 * @project    engineering-thesis
 * @brief      Auxiliary buffer structure automating UDP data parsing for LMS1xx driver class
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __SICK_LMS1XX_SCAN_DATA_BUFFER_H__
#define __SICK_LMS1XX_SCAN_DATA_BUFFER_H__

/* =========================================================== Includes =========================================================== */

// Standard includes
#include <cstdint>
// ROS includes
#include "rclcpp/rclcpp.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace sick::lms1xx {
    
/* ============================================================= Class ============================================================ */

/**
 * @brief Auxiliary buffer class providing automatic UDP data parsing for LMS1xx driver class
 */
class ScanDataBuffer
{
public: /* ------------------------------------------------ Public constants ------------------------------------------------------ */

    /// Size of the underlying buffer for UDP data batch (can be tuned to optimize performance)
    static constexpr std::size_t BUFFER_SIZE = 50000;

public: /* -------------------------------------------------- Public ctors -------------------------------------------------------- */

    /// Constructs the buffer
    ScanDataBuffer() = default;

    /**
     * @brief Construct a new ScanDataBuffer object with the given @p logger interface
     * 
     * @param logger 
     *    logger itnerface to be used
     */
    inline ScanDataBuffer(rclcpp::Logger &logger);
    
public: /* ------------------------------------------------- Public methods ------------------------------------------------------- */

    /**
     * @brief Reads data from the the given socket into the buffer
     * 
     * @param fd 
     *    file descriptor of the socket
     */
    inline void read(int fd);

    /**
     * @brief Parses current content of the buffer
     * 
     * @retval pointer
     *    to the beggining of the LIDAR message on success
     * @retval nullptr
     *    on failure
     */
    uint8_t *get_next_message();

    /**
     * @brief Removes first message in the buffer
     * 
     * @note The message to be removed needs to first be found with sucesfull call to
     *    @ref get_next_message()
     */
    inline void pop_last_buffer();

private: /* ------------------------------------------------ Private methods ------------------------------------------------------ */

    /**
     * @brief Shifts left content of the @a buffer so that @p new_start byte 
     *    is placed at buffer[0] after the call. Discards content that has been
     *    shifted out.
     * 
     * @param new_start 
     *    pointer to the desired start byte of the buffer
     */
    inline void shift_buffer(uint8_t* new_start);

private: /* ------------------------------------------------- Private data -------------------------------------------------------- */

    /// Optional logger interface
    rclcpp::Logger *logger { nullptr };

    /// Actual data buffer
    uint8_t buffer[BUFFER_SIZE];
    /// Current number of bytes held by the buffer
    std::size_t buffer_size { 0 };

    /// Auxiliary pointer pointing to the first byte in the @a buffer BEHIND the first LIDAR message
    uint8_t *first_message_end { nullptr };
    
};

/* ================================================================================================================================ */

} // End namespace sick::lms1xx

/* ==================================================== Implementation includes =================================================== */

#include "sick/lms1xx/scan_data_buffer/scan_data_buffer.hpp"

/* ================================================================================================================================ */

#endif
