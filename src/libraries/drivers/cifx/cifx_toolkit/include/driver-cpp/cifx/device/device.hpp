/* ============================================================================================================================ *//**
 * @file       driver.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 4th May 2022 12:03:11 pm
 * @modified   Friday, 27th May 2022 3:33:54 pm
 * @project    engineering-thesis
 * @brief      Definition of the RAII class wrapping description and providing related API for the 'Device' concept of the 
 *             CIFX Toolkit Framework
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __CIFX_DEVICE_DEVICE_H__
#define __CIFX_DEVICE_DEVICE_H__

/* =========================================================== Includes =========================================================== */

// System includes
#include <assert.h>
// Private includes
#include "cifx/device.hpp"
#include "cifx/common/conversions.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace cifx {

/* ======================================================= Static functions ======================================================= */

namespace conversions {
    
    /**
     * @brief Converts @p config structure to the C structure
     * 
     * @param config 
     *    configuration structure to be converted
     * @returns 
     *    @p config converted to C sturcture
     */
    static inline CIFX_DEVICE_INIT to_c(std::string_view name, const Device::Config &config) {
        return CIFX_DEVICE_INIT {
            .uio_num           = config.uio_num,
            .name              = name.data(),
            .irq_thread_params = to_c(config.irq_thread_params),
            .bootloader_file   = config.bootloader_file.string().data(),
            .firmware_file     = config.firmware_file.string().data(),
            .config_file       = config.config_file.string().data()
        };
    }

} // End namespace conversions

/* ===================================================== Public ctors & dtors ===================================================== */

Device::Device(
    Driver &driver,
    std::string_view name,
    const Config &config
) :
    driver{ driver }
{

    // Convert configuration to C structure
    auto c_config = conversions::to_c(name, config);
    
    // Else, add device to the Toolkit
    if(auto ret = xDeviceInit(&c_config); ret != CIFX_NO_ERROR){
        throw Error{ ret, "cifx::Device::Device", "Device initialization failed" };
    }

    // Keep name of the device
    this->name = std::string{ name };
    
    // Wait for IRQ thread of the device to start
    OS_Sleep(100);
}


Device::~Device() {
    
    /**
     * @note At the moment driver supports only one device, so of course, search is unnecessary
     */

    xDeviceDeinit();
}

/* ======================================================== Public methods ======================================================== */

Driver &Device::get_driver() {
    return driver;
}

/* ================================================================================================================================ */

} // End namespace cifx

/* ================================================================================================================================ */

#endif
