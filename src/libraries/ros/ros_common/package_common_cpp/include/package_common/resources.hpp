/* ============================================================================================================================ *//**
 * @file       resources.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 27th April 2022 1:56:00 pm
 * @modified   Thursday, 26th May 2022 2:14:44 am
 * @project    engineering-thesis
 * @brief      Declaration of helper utilities related to finding ament resources
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __PACKAGE_COMMON_RESOURCES_H__
#define __PACKAGE_COMMON_RESOURCES_H__

/* =========================================================== Includes =========================================================== */

// System includes
#include <filesystem>
#include <string>
// Private includes
#include "package_common/packages.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace package_common {

/**
 * @brief Subset of utilities related to ament @a resources
 */
namespace resources {

/* ======================================================== Exception types ======================================================= */

/**
 * @brief Error thrown when a resource is not found
 */
class ResourceNotFound : public std::out_of_range { 
public:

    /**
     * @param[in] resource_name 
     *    resource name used in the exception message
     * @param[in] msg_context (optional)
     *    optional ocntext string that the error message is prepended with (close in [] brackets) 
     */
    explicit inline ResourceNotFound(std::string_view resource_name, std::string_view msg_context = "");

    /// @brief Default virtual ctor
    virtual ~ResourceNotFound() = default;

private:

    /// Resource name used in the exception message
    const std::string resource_name;

};

/* ======================================================== Free functions ======================================================== */

/**
 * @brief Calculate absolute apth to the file named @p file residing in the directory
 *    described by the relative path stored in the @p resource marker file of the 
 *    @p package ament package
 * @details Function assumes that the ament package named @p package that is installed
 *    on the system (and can be found) registered resource named @p resource with the 
 *    content containing path to the directory containing the @p file (relative to the
 *    @p package prefix). This can be achieved inside the CMake file with
 * 
 *        ament_index_register_resource( <resource> CONTENT <relative_path_to_file_directory> )
 * 
 *    \f[
 *        K[n] = \frac{\begin{bmatrix} P[n|n-1](0, 0) \\ P[n|n-1](0, 1) \end{bmatrix}}{P[n|n-1](0, 0) + R}
 *    \f]
 * 
 * @param package 
 *    name of the resource package exporting the @p file
 * @param resource 
 *    name of the resource describing path to the directory containing @p file
 * @param file 
 *    name of the target file
 * @returns 
 *    absolute path to the requested file
 * 
 * @throws package_common::packages::PackageNotFound
 *    if given @p pakcage has not been found
 * @throws package_common::resources::ResourceNotFound
 *    if given @p resource has not been found
 * 
 * @see <a href="https://docs.ros.org/en/foxy/How-To-Guides/Ament-CMake-Documentation.html#adding-resources">Ament resources tutorial</a>
 */
std::filesystem::path get_file_path(
    std::string_view package, 
    std::string_view resource, 
    std::string_view file
);

/* ================================================================================================================================ */

} // End namespace resources
} // End namespace package_common

/* ==================================================== Implementation includes =================================================== */

#include "package_common/resources/resources.hpp"

/* ================================================================================================================================ */

#endif
