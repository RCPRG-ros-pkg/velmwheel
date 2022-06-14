/* ============================================================================================================================ *//**
 * @file       conversions.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Friday, 22nd April 2022 3:26:59 am
 * @modified   Friday, 27th May 2022 3:42:18 pm
 * @project    engineering-thesis
 * @brief      Declarations of private c-cpp conversions functions
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __CIFX_COMMON_CONVERSIONS_H__
#define __CIFX_COMMON_CONVERSIONS_H__

/* =========================================================== Includes =========================================================== */

// System includes
#include <sys/mman.h>
// System includes
#include <string_view>
#include <optional>
#include <chrono>
// Private includes
#include "cifx/config.hpp"
// CIFX includes
#include "cifxDriver.h"
#include "cifXHWFunctions.h"

/* ========================================================== Namespaces ========================================================== */

namespace cifx {
namespace conversions {

/* ========================================================== Conversions ========================================================= */

/**
 * @brief Converts @p log_level enumeration to the C constant
 * 
 * @param log_level 
 *    log level to be converted
 * @returns 
 *    @p log_level converted to C constant
 */
inline unsigned long to_c(LogLevel log_level);

/**
 * @brief Converts @p thread_policy to the C constant
 * 
 * @param thread_policy 
 *    log thread scheduling policy to be converted
 * @returns 
 *    @p thread_policy converted to C constant
 */
inline int to_c(ThreadSchedPolicy thread_policy);

/**
 * @brief Converts @p thread_info to the C structure
 * 
 * @param thread_info 
 *    log thread configuration to be converted
 * @returns 
 *    @p thread_info converted to C structure
 */
inline CIFX_THREAD_INFO to_c(ThreadConfig thread_info);

/* ================================================================================================================================ */

} // End namespace conversions
} // End namespace cifx

/* ==================================================== Implementation includes =================================================== */

#include "cifx/common/conversions/conversions.hpp"

/* ================================================================================================================================ */

#endif
