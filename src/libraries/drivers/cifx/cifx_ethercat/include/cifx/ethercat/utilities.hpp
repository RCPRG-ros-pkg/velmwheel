/* ============================================================================================================================ *//**
 * @file       utilities.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 27th April 2022 1:23:05 pm
 * @modified   Wednesday, 25th May 2022 10:09:54 pm
 * @project    engineering-thesis
 * @brief      Set of handy utilities for playing with CIFX EtherCAT driver library 
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __CIFX_ETHERCAT_UTILITIES_H__
#define __CIFX_ETHERCAT_UTILITIES_H__

/* =========================================================== Includes =========================================================== */

// System includes
#include <filesystem>
#include <string>
// Private includes
#include "package_common/resources.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace cifx::ethercat {

/* ======================================================== Free functions ======================================================== */

/**
 * @param name 
 *    name of the firmware file (either with or without '.nxf' suffix)
 * @returns 
 *    absolute path to the requested firmware file
 */
std::filesystem::path get_firmware_path(std::string_view name);

/* ================================================================================================================================ */

} // End namespace cifx::ethercat

#endif
