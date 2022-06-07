/* ============================================================================================================================ *//**
 * @file       packages.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 27th April 2022 2:05:04 pm
 * @modified   Wednesday, 25th May 2022 1:43:50 am
 * @project    engineering-thesis
 * @brief      Definitions of utilities related directly to ament packages
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __PACKAGE_COMMON_PACKAGES_PACKAGES_H__
#define __PACKAGE_COMMON_PACKAGES_PACKAGES_H__

/* =========================================================== Includes =========================================================== */

// System includes
#include <sstream>
// Pruvate includes
#include "package_common/packages.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace package_common {
namespace packages {

/* ======================================================= Helper functions ======================================================= */

namespace details {
    
    /**
     * @brief Formats 'package not found' error message
     * @param[in] package_name 
     *    package name used in the exception message
     * @param[in] msg_context (optional)
     *    optional ocntext string that the error message is prepended with (close in [] brackets) 
     */
    static inline std::string format_package_not_found_error_message(
        std::string_view package_name,
        std::string_view msg_context
    ) {

        std::stringstream message_builder;
        
        // Check if context string given
        if(msg_context.empty())
            message_builder << "[" << msg_context << "] ";

        // Format main message
        message_builder << "Package '" << package_name << "' not found, searching: [";

        // Get paths that has been searched
        auto search_paths = ament_index_cpp::get_search_paths();
        // List search paths
        for (const auto & path : search_paths)
            message_builder << path + ", ";

        // Convert s-stream to string
        std::string message = message_builder.str();

        // Remove trailing comma
        if (search_paths.size() > 0)
            message = message.substr(0, message.size() - 2);
        
        // Append closing brakcet
        message.append("]");

        return message;
    }

}

/* ========================================================== Error types ========================================================= */

PackageNotFound::PackageNotFound(std::string_view package_name, std::string_view msg_context) :
    std::out_of_range{ details::format_package_not_found_error_message(package_name, msg_context) },
    package_name{ package_name }
{ }

/* ================================================================================================================================ */

} // End namespace packages
} // End namespace package_common

#endif
