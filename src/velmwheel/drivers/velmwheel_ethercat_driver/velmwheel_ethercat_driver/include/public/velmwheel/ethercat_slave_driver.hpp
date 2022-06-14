/* ============================================================================================================================ *//**
 * @file       ethercat_driver_impl.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 28th April 2022 1:21:00 pm
 * @modified   Monday, 13th June 2022 8:58:39 pm
 * @project    engineering-thesis
 * @brief      Definition of the abstract interface for loadable EtherCAT slave-driver plugins
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __VELMWHEEL_ETHERCAT_SLAVE_DRIVER_IMPL_H__
#define __VELMWHEEL_ETHERCAT_SLAVE_DRIVER_IMPL_H__

/* =========================================================== Includes =========================================================== */

// ROS includes
#include "rclcpp/rclcpp.hpp"
// Private includes
#include "cifx/ethercat.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace velmwheel {

/* ========================================================== Definitions ========================================================= */

/**
 * @brief Interface of the plugin class implementing driver of the EtherCAT bus slave device
 */
class EthercatSlaveDriver {

public: /* --------------------------------------------- Public implementation API ----------------------------------------------- */

    /**
     * @brief Initialization routine of the driver. At this step implementation can register 
     *    ROS-specific interfaces like topics, subscriptions and parameters declarations
     * 
     * @param node 
     *    handle to the ROS node interface
     * @returns 
     *    function should return list of names of the slave devices that this driver wants to manage
     */
    virtual std::vector<std::string> initialize(rclcpp::Node &node) = 0;

    /**
     * @brief Function called by the loading process after @ref initialize . Provides driver
     *    with handle to the interface enabling slave device management
     * 
     * @param slaves 
     *    list of handles to the slave devices interfaces
     */
    virtual void configure(std::vector<cifx::ethercat::Slave*> slaves) = 0;

public: /* ---------------------------------------------------- Public members --------------------------------------------------- */

    /// @brief Destructs the driver
    virtual ~EthercatSlaveDriver() = default;

protected: /* ------------------------------------------------ Protected members ------------------------------------------------- */

    /// @brief Constructs the driver
    EthercatSlaveDriver() = default;

};

/* ================================================================================================================================ */

} // End namespace velmwheel

/* ================================================================================================================================ */

#endif
