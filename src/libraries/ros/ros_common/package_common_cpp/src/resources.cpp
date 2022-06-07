/* ============================================================================================================================ *//**
 * @file       resources.cpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 27th April 2022 2:30:13 pm
 * @modified   Wednesday, 25th May 2022 1:50:01 am
 * @project    engineering-thesis
 * @brief      Definitions of helper utilities related to finding ament resources
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

/* =========================================================== Includes =========================================================== */

// Ament idnex includes
#include "ament_index_cpp/get_package_prefix.hpp"
#include "ament_index_cpp/get_resource.hpp"
// Private includes
#include "package_common/resources.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace package_common {
namespace resources {

/* ======================================================== Free functions ======================================================== */

std::filesystem::path get_file_path(
    std::string_view package, 
    std::string_view resource, 
    std::string_view file
) {

    std::string package_prefix;

    // Find bootloader resource in the ament index
    try {
        package_prefix = ament_index_cpp::get_package_prefix(std::string{ package });
    } catch(...) {
        throw packages::PackageNotFound{ package, "package_common::resources::get_file_path" };
    }

    std::string resource_files_path;

    // Find bootloader resource in the ament index
    if(not ament_index_cpp::get_resource(std::string{ resource }, std::string{ package }, resource_files_path))
        throw ResourceNotFound{ resource, "package_common::resources::get_file_path" };

    // Compile path to the bootloader
    auto file_path = 
        std::filesystem::path{ package_prefix      } /
        std::filesystem::path{ resource_files_path } / 
        std::filesystem::path{ file                };

    return file_path;
}

/* ================================================================================================================================ */

} // End namespace resources
} // End namespace package_common
