/* ============================================================================================================================ *//**
 * @file       utilities.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 27th April 2022 1:23:05 pm
 * @modified   Friday, 27th May 2022 5:09:11 pm
 * @project    engineering-thesis
 * @brief      Set of handy utilities for playing with CIFX driver library
 *     
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __CIFX_UTILITIES_H__
#define __CIFX_UTILITIES_H__

/* =========================================================== Includes =========================================================== */

// System includes
#include <filesystem>
#include <string>
// Private includes
#include "package_common/resources.hpp"
#include "cifx/config.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace cifx {

/* ======================================================== Free functions ======================================================== */

/**
 * @param name 
 *    name of the bootloader file (either with or without '.bin' suffix)
 * @returns 
 *    absolute path to the requested bootloader file
 */
std::filesystem::path get_bootloader_path(std::string_view name);

/**
 * @brief Configures parameters of the current thread
 * 
 * @param config 
 *    configruation to be set
 * 
 * @throws std::runtime_error 
 *    if configuration could not be set
 */
inline void set_thread_params(const ThreadConfig &config);

/**
 * @brief Configures parameters of the target @p thread
 * 
 * @param thread 
 *    ID of the target thread 
 * @param config 
 *    configruation to be set
 * 
 * @throws std::runtime_error 
 *    if configuration could not be set
 */
void set_thread_params(thread_id thread, const ThreadConfig &config);

/* ================================================================================================================================ */

} // End namespace cifx

/* ==================================================== Implementation includes =================================================== */

#include "cifx/utilities/utilities.hpp"

/* ================================================================================================================================ */

#endif
