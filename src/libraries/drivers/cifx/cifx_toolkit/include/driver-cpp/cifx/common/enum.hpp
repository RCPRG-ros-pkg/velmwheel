/* ============================================================================================================================ *//**
 * @file       common.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 28th April 2022 4:15:35 pm
 * @modified   Wednesday, 25th May 2022 9:36:44 pm
 * @project    engineering-thesis
 * @brief      Set of common enumeration-related utilities
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __CIFX_UTILITIES_ENUM_H__
#define __CIFX_UTILITIES_ENUM_H__

/* =========================================================== Includes =========================================================== */

#include <type_traits>

/* ========================================================== Namespaces ========================================================== */

namespace cifx {
namespace common {

/* ======================================================= Helper functions ======================================================= */

/**
 * @brief Converts enum value to it's underlying representation
 * 
 * @tparam Enum 
 *    source enum type
 * @param e 
 *    value to be converted
 * @returns 
 *    enum value converted to it's underlying representation
 */
template<typename Enum>
constexpr std::underlying_type_t<Enum> to_underlying(Enum e) {
    return static_cast<std::underlying_type_t<Enum>>(e);
}

/**
 * @brief Converts value to it's corresopnding enum representation
 * 
 * @tparam Enum 
 *    target enum type
 * @param val 
 *    value to be converted
 * @returns 
 *    value converted to it's coresponding enum representation
 */
template<typename Enum>
constexpr Enum to_enum(std::underlying_type_t<Enum> val) {
    return static_cast<Enum>(val);
}

/* ================================================================================================================================ */

} // End namespace common
} // End namespace cifx

/* ================================================================================================================================ */

#endif
