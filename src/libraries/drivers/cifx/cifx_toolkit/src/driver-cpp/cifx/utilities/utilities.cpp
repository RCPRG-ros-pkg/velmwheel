/* ============================================================================================================================ *//**
 * @file       utilities.cpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 27th April 2022 1:23:05 pm
 * @modified   Tuesday, 14th June 2022 4:13:31 pm
 * @project    engineering-thesis
 * @brief      Set of handy utilities for playing with CIFX driver library 
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

/* =========================================================== Includes =========================================================== */

// Standard includes
#include <sstream>
// Package includes
#include "ament_index_cpp/get_package_prefix.hpp"
#include "ament_index_cpp/get_resource.hpp"
// Private includes
#include "cifx/error.hpp"
#include "cifx/utilities.hpp"
#include "cifx/common/conversions.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace cifx {

/* ======================================================== Free functions ======================================================== */

std::filesystem::path get_bootloader_path(std::string_view name) {

    // Name of the ament resource
    constexpr auto ament_resource_name = "bootloaders";
    // Name of the package
    constexpr auto package_name = "cifx_toolkit";
    // File-format suffix of the bootloader file
    constexpr auto bootloader_suffix = ".bin";

    // Get absolute path to the file
    std::string bootloaders_path = package_common::resources::get_file_path(package_name, ament_resource_name, name);
    // Append file format suffix to the @p name
    std::string bootloaders_path_suffixed = name.ends_with(bootloader_suffix) ? 
        bootloaders_path : (bootloaders_path + bootloader_suffix);

    return std::filesystem::path{ std::move(bootloaders_path_suffixed) };
}


void set_thread_params(thread_id thread, const ThreadConfig &config) {

    struct sched_param param;

    // Prepare scheduling configruation
    param.sched_priority = config.sched_priority;
    // Try to configure
    if(pthread_setschedparam(thread, conversions::to_c(config.sched_policy), &param) != 0) {
        std::stringstream ss;
        ss << "[cifx::set_thread_params] Failed to configure scheduling policy for thread "
           << "(" << "ID: " << thread << ")";
        throw std::runtime_error{ ss.str() };
    }

    cpu_set_t cs;

    // Zero CPU affinity configuration
    CPU_ZERO(&cs);

    constexpr std::size_t BITS_IN_BYTE = 8;

    // Set bits corresponding to requested affinity
    for(unsigned i = 0; i < BITS_IN_BYTE * sizeof(config.affinity); i++)
        if(config.affinity & (1 << i))
            CPU_SET(i, &cs);

    // Configure thread's affinity
    if(pthread_setaffinity_np(thread, sizeof(cs), &cs) != 0) {
        std::stringstream ss;
        ss << "[cifx::set_thread_params] Failed to configure affinity for thread "
           << "(" << "ID: " << thread << ")";
        throw std::runtime_error{ ss.str() };
    }

}

/* ================================================================================================================================ */

} // End namespace cifx
