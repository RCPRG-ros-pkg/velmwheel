/* ============================================================================================================================ *//**
 * @file       format.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Monday, 4th July 2022 3:06:48 pm
 * @modified   Monday, 4th July 2022 8:45:57 pm
 * @project    engineering-thesis
 * @brief      Definitions related to the format of LMS1xx command messages
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __SICK_LMS1XX_FORMAT_H__
#define __SICK_LMS1XX_FORMAT_H__

/* =========================================================== Includes =========================================================== */

// Standard includes
#include <cstdint>

/* ========================================================== Namespaces ========================================================== */

namespace sick::lms1xx {

/* =========================================================== Constants ========================================================== */

/// Byte code of the <STX> marker of the LIDAR communciation protocol (see documentation)
constexpr uint8_t STX_MARKER = 0x02;
/// Byte code of the <ETX> marker of the LIDAR communciation protocol (see documentation)
constexpr uint8_t ETX_MARKER = 0x03;

/* ================================================================================================================================ */

} // End namespace sick::lms1xx

/* ================================================================================================================================ */

#endif
