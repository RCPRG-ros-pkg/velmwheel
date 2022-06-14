/* ============================================================================================================================ *//**
 * @file       driver.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 4th May 2022 11:57:09 am
 * @modified   Friday, 27th May 2022 3:42:15 pm
 * @project    engineering-thesis
 * @brief      Definition of the RAII class wrapping an connection-point (called 'Driver') to the CIFX Toolkit Framework
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __CIFX_DRIVER_HPP__
#define __CIFX_DRIVER_HPP__

/* =========================================================== Includes =========================================================== */

// System includes
#include <string>
#include <stdexcept>
#include <map>
#include <vector>
#include <cstdint>
// Private includes
#include "cifx/error.hpp"
#include "cifx/config.hpp"
// CIFX includes
#include "cifxDriver.h"
#include "cifXHWFunctions.h"

/* ========================================================== Namespaces ========================================================== */

namespace cifx {

/* ============================================================= Class ============================================================ */

/**
 * @brief RAII class wrapping an connection-point (called 'Driver') to the CIFX 
 *    Toolkit Framework
 */
class Driver {

public: /* ------------------------------------------------------ Public types ---------------------------------------------------- */

    /// Local typename for the CIFX Driver handle
    using Handle = CIFXHANDLE;

    /**
     * @brief Initialization structure for CIFX proprietary driver
     */
    struct Config {
        
        /// COS polling interval in milliseconds (no-value to disable)
        std::optional<std::chrono::milliseconds> cos_polling_interval_ms { };
        /// COS-polling thread's parameters (taken into account if polling enabled)
        ThreadConfig cos_polling_thread_params;
        
        /// Tracing logs threshold of driver
        LogLevel trace_level { LogLevel::Info };

    };

public: /* -------------------------------------------------- Public constructors ------------------------------------------------- */

    /**
     * @brief Constructs a new Driver entry-point to the CIFX Toolkit Framework
     * 
     * @param config 
     *    driver's configuration 
     * 
     * @throws Error
     *    when the procedure failed to acquire the driver handle
     */
    inline Driver(const Config &config);

    /**
     * @brief Destroys the Driver entry-point to the CIFX Toolkit Framework
     */
    inline ~Driver();

public: /* ---------------------------------------------------- Public operators -------------------------------------------------- */

    /**
     * @brief Converts DriverHandle object into it's C-style representation
     */
    inline operator Handle() const;

private: /* ----------------------------------------------- Private member variables ---------------------------------------------- */

    /// Handle to the wrapped entry point
    Handle handle;

};

/* ================================================================================================================================ */

} // End namespace cifx

/* ==================================================== Implementation includes =================================================== */

#include "cifx/driver/driver.hpp"

/* ================================================================================================================================ */


#endif
