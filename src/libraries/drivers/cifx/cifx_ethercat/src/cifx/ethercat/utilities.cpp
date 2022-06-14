/* ============================================================================================================================ *//**
 * @file       utilities.cpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 27th April 2022 1:23:05 pm
 * @modified   Wednesday, 25th May 2022 10:09:21 pm
 * @project    engineering-thesis
 * @brief      Set of handy utilities for playing with CIFX driver library 
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

/* =========================================================== Includes =========================================================== */

// Package includes
#include "ament_index_cpp/get_package_prefix.hpp"
#include "ament_index_cpp/get_resource.hpp"
// Private includes
#include "cifx/error.hpp"
#include "cifx/ethercat/utilities.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace cifx {
namespace ethercat {

/* ======================================================== Free functions ======================================================== */

std::filesystem::path get_firmware_path(std::string_view name) {

    // Name of the ament resource
    constexpr auto ament_resource_name = "firmware";
    // Name of the package
    constexpr auto package_name = "cifx_ethercat";
    // File-format suffix of the firmware file
    constexpr auto firmware_suffix = ".nxf";

    // Get absolute path to the file
    std::string firmwares_path = package_common::resources::get_file_path(package_name, ament_resource_name, name);
    // Append file format suffix to the @p name
    std::string firmwares_path_suffixed = name.ends_with(firmware_suffix) ? 
        firmwares_path : (firmwares_path + firmware_suffix);

    return std::filesystem::path{ std::move(firmwares_path_suffixed) };
}

/* ================================================================================================================================ */

} // End namespace ethercat
} // End namespace cifx
