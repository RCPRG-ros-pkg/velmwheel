/* ============================================================================================================================ *//**
 * @file       driver.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 4th May 2022 12:03:11 pm
 * @modified   Friday, 27th May 2022 3:42:15 pm
 * @project    engineering-thesis
 * @brief      Definition of the RAII class wrapping an connection-point (called 'Driver') to the CIFX Toolkit Framework
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __CIFX_DRIVER_DRIVER_H__
#define __CIFX_DRIVER_DRIVER_H__

/* =========================================================== Includes =========================================================== */

// Private includes
#include "cifx/driver.hpp"
#include "cifx/common/conversions.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace cifx {

/* ======================================================= Helper functions ======================================================= */

namespace conversions {

    /**
     * @brief Converts @p config structure to the C structure
     * 
     * @param config 
     *    configuration structure to be converted
     * @returns 
     *    @p config converted to C sturcture
     */
    static inline CIFX_LINUX_INIT to_c(const Driver::Config &config) {
        return CIFX_LINUX_INIT {
            .cos_polling_interval_ms   = static_cast<int>(config.cos_polling_interval_ms.has_value() ? config.cos_polling_interval_ms->count() : -1),
            .cos_polling_thread_params = conversions::to_c(config.cos_polling_thread_params),
            .trace_level        = conversions::to_c(config.trace_level),
        };
    }

} // End namespace conversions

/* ===================================================== Public ctors & dtors ===================================================== */

Driver::Driver(const Config &config) {

    // Convert configuration to C structure
    auto c_config = conversions::to_c(config);

    // Initialize CIFX/netX Toolkit
    if(auto ret = xToolkitInit(&c_config); ret != CIFX_NO_ERROR) {
        throw Error{ ret, "cifx::Driver::Driver", "Toolkit initialization failed" };
    };   

    // Open handle to the driver
    if(auto ret = xDriverOpen(static_cast<CIFXHANDLE*>(&handle)); ret != CIFX_NO_ERROR)
        throw Error { ret, "cifx::Driver::Driver", "Failed to open a new instance of the CIFX Driver" };
}


Driver::~Driver() {

    // Close handle to the driver
    if(auto ret = xDriverClose(static_cast<CIFXHANDLE>(handle)); ret != CIFX_NO_ERROR)
        xTraceError("cifx::Driver::~Driver", " Failed to close an instance of the CIFX Driver (%s)", error_to_str(ret));

    // Deinitialize the toolkit
    xToolkitDeinit();
}

/* ======================================================= Public operators ======================================================= */

Driver::operator Handle() const {
    return handle;
}

/* ================================================================================================================================ */

} // End namespace cifx

/* ================================================================================================================================ */

#endif
