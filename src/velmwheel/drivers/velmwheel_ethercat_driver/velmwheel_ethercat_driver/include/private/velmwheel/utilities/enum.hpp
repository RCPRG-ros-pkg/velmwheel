/* ============================================================================================================================ *//**
 * @file       common.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 28th April 2022 4:15:35 pm
 * @modified   Friday, 27th May 2022 4:56:50 pm
 * @project    engineering-thesis
 * @brief      Set of common utilities
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __VELMWHEEL_UTILITIES_ENUM_H__
#define __VELMWHEEL_UTILITIES_ENUM_H__

/* =========================================================== Includes =========================================================== */

#include <type_traits>

/* ========================================================== Namespaces ========================================================== */

namespace velmwheel {
namespace utilities {

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

} // End namespace utilities
} // End namespace velmwheel

#endif
