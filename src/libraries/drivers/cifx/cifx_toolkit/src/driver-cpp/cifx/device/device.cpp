/* ============================================================================================================================ *//**
 * @file       driver.cpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 4th May 2022 12:03:11 pm
 * @modified   Wednesday, 25th May 2022 9:45:34 pm
 * @project    engineering-thesis
 * @brief      Definition of the RAII class wrapping description and providing related API for the 'Device' concept of the 
 *             CIFX ToolkitFramework
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

/* =========================================================== Includes =========================================================== */

// System includes
#include <assert.h>
// Private includes
#include "cifx/device.hpp"
#include "cifx/common/conversions.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace cifx {

/* ======================================================== Public methods ======================================================== */

std::filesystem::path Device::get_config_file(std::size_t id) const {
    
    assert(id == 0);

    // Get pointer to the CIFX device registered in the toolkit
    PDEVICEINSTANCE cifx_device = xDeviceGet();
    // Check if device has been proeprly initialized
    if(!cifx_device)
        throw cifx::Error{ CIFX_INVALID_POINTER, "cifx::Device::get_config_file Method could not obtain instance of the device from the Toolkit" };

    CIFX_DEVICE_INFORMATION dev_info { };

    // Prepare device information structure for the toolkit
    dev_info.ulDeviceNumber   = cifx_device->ulDeviceNumber;
    dev_info.ulSerialNumber   = cifx_device->ulSerialNumber;
    dev_info.ulChannel        = 0;
    dev_info.ptDeviceInstance = cifx_device;

    /**
     * @note @a dev_info.ulChannel is set to @c 0 as the current implementation of CIFX Toolkit supports
     *    only @c 0'th channel of the CIFX device
     */

    CIFX_FILE_INFORMATION file_info { };
    
    // Try to get configuration file
    if(auto ret = USER_GetConfigurationFile(&dev_info, id, &file_info); !ret)
        throw cifx::Error{ CIFX_FUNCTION_FAILED, "cifx::Device::get_config_file Failed to obtain configuration for the CIFX device" };

    // Return path to the file the file
    return std::filesystem::path{ file_info.szFullFileName };
}

/* ================================================================================================================================ */

} // End namespace cifx

