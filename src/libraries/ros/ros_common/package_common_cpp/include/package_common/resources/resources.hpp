/* ============================================================================================================================ *//**
 * @file       resources.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 27th April 2022 2:05:04 pm
 * @modified   Wednesday, 25th May 2022 1:43:06 am
 * @project    engineering-thesis
 * @brief      Definitions of helper utilities related to finding ament resources
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __PACKAGE_COMMON_RESOURCES_RESOURCES_H__
#define __PACKAGE_COMMON_RESOURCES_RESOURCES_H__

/* =========================================================== Includes =========================================================== */

// System includes
#include <sstream>
// Pruvate includes
#include "package_common/resources.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace package_common {
namespace resources {

/* ======================================================= Helper functions ======================================================= */

namespace details {
    
    /**
     * @brief Formats 'resource not found' error message
     * @param[in] resource_name 
     *    resource name used in the exception message
     * @param[in] msg_context (optional)
     *    optional ocntext string that the error message is prepended with (close in [] brackets) 
     */
    static inline std::string format_resource_not_found_error_message(
        std::string_view resource_name,
        std::string_view msg_context
    ) {

        std::stringstream message_builder;
        
        // Check if context string given
        if(msg_context.empty())
            message_builder << "[" << msg_context << "] ";

        // Format main message
        message_builder << "Resource '" << resource_name << "' not found";

        return message_builder.str();
    }

}

/* ========================================================== Error types ========================================================= */

ResourceNotFound::ResourceNotFound(std::string_view resource_name, std::string_view msg_context) :
    std::out_of_range{ details::format_resource_not_found_error_message(resource_name, msg_context) },
    resource_name{ resource_name }
{ }

/* ================================================================================================================================ */

} // End namespace resources
} // End namespace package_common

#endif
