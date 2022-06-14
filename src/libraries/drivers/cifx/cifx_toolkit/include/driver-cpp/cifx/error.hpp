/* ============================================================================================================================ *//**
 * @file       error.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Monday, 14th February 2022 2:33:20 pm
 * @modified   Wednesday, 25th May 2022 9:40:59 pm
 * @project    engineering-thesis
 * @brief      Declarations of helper functions and types associated with CIFX errors system
 *     
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __CIFX_ERROR_H__
#define __CIFX_ERROR_H__

/* =========================================================== Includes =========================================================== */

// System includes
#include <string>
#include <stdexcept>
#include <map>
#include <vector>
#include <cstdint>
// CIFX includes
#include "cifxDriver.h"

/* ============================================================ Macros ============================================================ */

/*
 * Additional error code compliant with `cifxErrors.h`
 */

/// Error code returned when too many CIFX devices are registered to the toolkit to add another one
#define CIFX_TKT_TOO_MANY_DEVICES ((int32_t)0xC0001006L)
/// Error code returned when non-registered device is referenced by the user
#define CIFX_TKT_NO_DEVICE        ((int32_t)0xC0001007L)

/* ========================================================== Namespaces ========================================================== */

namespace cifx {

/* ======================================================== Free functions ======================================================== */

/**
 * @brief Converts CIFX error code @p error to the human-readable string
 * 
 * @param error 
 *    error code to be converted
 * 
 * @throws std::out_of_range 
 *    if invalid @p error has been given
 */
inline std::string error_to_str(int32_t error);

/* ============================================================= Types ============================================================ */

/**
 * @brief Dedicated exception type for the Toolkit-specific errors
 */
class Error : public std::runtime_error {
public:

    /// Constructs Error with the given @p code and @p what message
    inline Error(int32_t code, const std::string &what);

    /// Constructs Error with the given @p code , @p context @p what message
    inline Error(int32_t code, std::string_view context, const std::string &what);

    /// Returns error code associated with the exception
    inline int32_t code() const;

private:

    /// Code associated with the error
    int32_t error_code;
    
};

/* ================================================================================================================================ */

} // End namespace cifx

/* ==================================================== Implementation includes =================================================== */

#include "cifx/error/error.hpp"

/* ================================================================================================================================ */

#endif
