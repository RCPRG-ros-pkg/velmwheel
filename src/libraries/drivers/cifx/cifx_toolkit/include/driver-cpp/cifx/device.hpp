/* ============================================================================================================================ *//**
 * @file       device.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 4th May 2022 12:10:47 pm
 * @modified   Friday, 27th May 2022 3:40:54 pm
 * @project    engineering-thesis
 * @brief      Definition of the RAII class wrapping description and providing related API for the 'Device' concept of the CIFX 
 *             Toolkit Framework
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __CIFX_DEVICE_H__
#define __CIFX_DEVICE_H__

/* =========================================================== Includes =========================================================== */

// System includes
#include <filesystem>
// Private includes
#include "cifx/config.hpp"
#include "cifx/error.hpp"
#include "cifx/driver.hpp"
// CIFX includes
#include "cifxDriver.h"
#include "cifXHWFunctions.h"

/* ========================================================== Namespaces ========================================================== */

namespace cifx {
    
/* ============================================================= Class ============================================================ */

/**
 * @brief RAII class wrapping description and providing related API for the 'Device' concept 
 *    of the CIFX Toolkit Framework
 */
class Device {

    /// Make Channel class a friend to let it access Device's specification
    friend class Channel;

public: /* ------------------------------------------------------ Public types ---------------------------------------------------- */

    /**
     * @brief Driver-specific structure describing initial parameters of the CIFX device
     */
    struct Config {
        
        /// Index of the uioX device representing the CIFX card
        int uio_num;
        
        /// IRQ-handling thread's parameters
        ThreadConfig irq_thread_params;

        /// Path to the bootloader file (default path used if NULL)
        std::filesystem::path bootloader_file;
        /// Path to the firmware file (default path used if NULL)
        std::filesystem::path firmware_file;
        /// Path to the card's configuration (e.g. ENI) file (default path used if NULL, no configuration file if "none")
        std::filesystem::path config_file;
        
    };

public: /* -------------------------------------------------- Public constructors ------------------------------------------------- */

    /**
     * @brief Constructs a new object class wrapping description and providing related API 
     *     for the 'Device' concept of the CIFX Toolkit Framework
     * 
     * @param device 
     *    handle to the device to register device for ( provided in the @ref DeviceConfig structure )
     * @param name 
     *    Device's name inside toolkit
     * @param config 
     *    configuration of the device
     * 
     * @throws Error
     *    when the procedure failed to acquire the driver handle
     */
    inline Device(Driver &driver, std::string_view name, const Config &config);

    /**
     * @brief Destroys the Device object unregistering related device from the Toolkit
     */
    inline ~Device();

public: /* ----------------------------------------------------- Public methods --------------------------------------------------- */

    /**
     * @returns 
     *    reference to the associated driver
     */
    inline Driver &get_driver();

    /**
     * @param id
     *    index of the configuration file to be 
     * @returns 
     *    path to the currently loaded configuration file of the device
     * 
     * @note At the moment, the underlying CIF Toolkit supports only 
     *    configuration file with index @c 0
     */
    std::filesystem::path get_config_file(std::size_t id = 0) const;

private: /* ----------------------------------------------- Private member variables ---------------------------------------------- */

    /// Handle to the related Driver
    Driver &driver;
    /// Device's name inside toolkit
    std::string name;

};

/* ================================================================================================================================ */

} // End namespace cifx

/* ==================================================== Implementation includes =================================================== */

#include "cifx/device/device.hpp"

/* ================================================================================================================================ */


#endif
