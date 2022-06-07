/* ============================================================================================================================ *//**
 * @file       packages.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 27th April 2022 2:05:04 pm
 * @modified   Wednesday, 25th May 2022 1:46:06 am
 * @project    engineering-thesis
 * @brief      Declarations of utilities related directly to ament packages
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __PACKAGE_COMMON_ERROR_H__
#define __PACKAGE_COMMON_ERROR_H__

/* =========================================================== Includes =========================================================== */

// System includes
#include <string>
#include <stdexcept>
#include <sstream>
// Ament index includes
#include "ament_index_cpp/get_search_paths.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace package_common {

/**
 * @brief Subset of utilities related directly to ament packages
 */
namespace packages {

/* ========================================================== Error types ========================================================= */

/**
 * @brief Error thrown when a package is not found
 */
class PackageNotFound : public std::out_of_range { 
public:

    /**
     * @param[in] package_name 
     *    package name used in the exception message
     * @param[in] msg_context (optional)
     *    optional ocntext string that the error message is prepended with (close in [] brackets) 
     */
    explicit inline PackageNotFound(std::string_view package_name, std::string_view msg_context = "");

    /// @brief Default virtual ctor
    virtual ~PackageNotFound() = default;

private:

    /// Package name used in the exception message
    const std::string package_name;
    
};

/* ================================================================================================================================ */

} // End namespace packages
} // End namespace package_common

/* ==================================================== Implementation includes =================================================== */

#include "package_common/packages/packages.hpp"

/* ================================================================================================================================ */

#endif
