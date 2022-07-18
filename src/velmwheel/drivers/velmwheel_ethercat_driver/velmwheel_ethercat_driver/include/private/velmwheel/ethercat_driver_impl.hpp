/* ============================================================================================================================ *//**
 * @file       ethercat_driver.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 28th April 2022 4:00:25 pm
 * @modified   Wednesday, 29th June 2022 12:03:51 pm
 * @project    engineering-thesis
 * @brief      Definition of the class implementing bus-driving routines of the velmwheel_ethercat_driver node
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __VELMWHEEL_ETHERCAT_DRIVER_IMPL_H__
#define __VELMWHEEL_ETHERCAT_DRIVER_IMPL_H__

/* =========================================================== Includes =========================================================== */

// System includes
#include <type_traits>
// ROS includes
#include "pluginlib/class_loader.hpp"
// Private includes
#include "cifx/ethercat.hpp"
#include "velmwheel/ethercat_slave_driver.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace velmwheel {

/* ======================================================== Auxiliary types ======================================================= */

/**
 * @brief Process memory locking schemes
 */
enum class MemoryLockScheme {

    // No memory pages locked
    None,
    // Current memory pages locked
    Current,
    // Current and future memory pages locked
    All
    
};

/// Alias for the CIFX toolkit-specific thread configuration structure
using ThreadConfig = cifx::ThreadConfig;
/// Alias for the CIFX toolkit-specific thread scheduling configuration
using ThreadSchedPolicy = cifx::ThreadSchedPolicy;

namespace ethercat {

    /*
     * Type aliases defined to decouple ROS interface from driver library definitions
     */

    /// State of the EtherCAT Master device 
    using MasterState = cifx::ethercat::Master::State;
    /// Extended state info of the EtherCAT Master device 
    using MasterExtendedState = cifx::ethercat::Master::ExtendedState;
    /// State info of the EtherCAT Master device 
    using MasterStateInfo = cifx::ethercat::Master::StateInfo;
    /// Ethercat driver log level
    using LogLevel = cifx::LogLevel;
    /// Bus timing parameters
    using TimingInfo = cifx::ethercat::Master::TimingInfo;

}

/* ============================================================= Class ============================================================ */

/**
 * @class EthercatDriverImpl
 * @brief Implementation of bus-control routines utilized by the velmwheel_ethercat_driver node
 */
class EthercatDriverImpl {

public: /* --------------------------------------------------- Public types ------------------------------------------------------- */


    /**
     * @brief Configuration of the driver
     */
    struct Config {

        /// Path to the target ENI configuration
        std::filesystem::path eni_source;

        /// Timeout for all slaves being brought into the Operational state
        std::optional<std::chrono::milliseconds> slaves_up_timeout;

        /**
         * @brief Initial configuration of the process
         */
        struct {

            /// Memory allocation config
            struct {

                /// Initial configuration of memory locking
                MemoryLockScheme lock_scheme { MemoryLockScheme::None };
                /// Whether to disable returning memory to the system with sbrk()
                bool disable_trimming { false };
                /// Whether to disable mmap()
                bool disable_mmap { false };

            } memory_allocation;

            /// Configuration of the main thread
            ThreadConfig processing_thread;

        } process_config;

        /**
         * @brief Configuration of the CIFX-specific parameters
         */
        struct {

            /// Log level of the underlying driver
            ethercat::LogLevel log_level;

            /// Polling interval of the CoS-polling CIFX Toolkit thread in [s] (polling disabled if not given)
            std::optional<std::chrono::milliseconds> cos_polling_interval;

            /// Configuration of the polling thread
            ThreadConfig cos_polling_thread_config;
            /// Configuration of the IRQ-handling thread
            ThreadConfig irq_thread_config;

        } driver_config;

    };

    /**
     * @brief Helper enumeration of events that can be reported by the bus driver
     */
    enum class Event {

        /**
         * Event reported by the bus driver when the new input data is received from 
         * the fieldbus into the reception buffer and can be read by application
         */
        ReadBusReady,
        
        /**
         * Event reported by the bus driver when the new output data can be written 
         * to the transmission buffer
         */
        WriteBusRead
        
    };

    /**
     * @brief Structure describing a loaded driver
     */
    struct DriverDescriptor {

        // Unique ID of the slave driver
        uint64_t id;
        // Name of the driver's plugin
        std::string plugin_name;

    };

public: /* ----------------------------------------------- Public ctors & dtors --------------------------------------------------- */

    /**
     * @brief Construct a new EthercatDriverImpl oobject with the given configuration
     * 
     * @param node 
     *    reference to the node interface
     * @param config 
     *    configuration of the driver
     */
    EthercatDriverImpl(
        rclcpp::Node &node,
        const Config &config
    );

    /// @brief Default destructor
    ~EthercatDriverImpl() = default;

public: /* -------------------------------------- Public methods (process configuration) ------------------------------------------ */

    /**
     * @brief Locks process memory according to the given scheme
     * 
     * @param scheme 
     *    memory locking scheme
     * 
     * @retval true 
     *    on succes
     * @retval false 
     *    on failure
     */
    inline bool lock_memory(MemoryLockScheme scheme);

    /**
     * @brief Disables returning memory to the system with @ref sbrk()
     * 
     * @retval true 
     *    on succes
     * @retval false 
     *    on failure
     */
    inline bool disable_memory_trimming();

    /**
     * @brief Disables mmap() usage in memory allocation scheme
     * 
     * @retval true 
     *    on succes
     * @retval false 
     *    on failure
     */
    inline bool disable_mmap();

public: /* ------------------------------------------ Public methods (device drivers) --------------------------------------------- */

    /**
     * @brief Tries to load driver plugin for the EtherCAT slave device
     * 
     * @param plugin 
     *    name of the plugin
     * @returns 
     *    unique ID of the loaded driver on success
     * 
     * @throws pluginlib::PluginlibException 
     *    if the driver could not be loaded
     * @throws ...
     *    anything thrown during driver initialization and registration
     */
    uint64_t load_driver(std::string_view plugin);

    /**
     * @brief Tries to unload driver plugin from the node
     * 
     * @param id 
     *    ID of the driver to be unloaded
     * 
     * @throws std::runtime_error 
     *    if the driver could not be unloaded
     */
    void unload_driver(uint64_t id);

    /**
     * @returns 
     *    list of descriptions of currently loaded drivers
     */
    std::vector<DriverDescriptor> list_drivers() const;

public: /* ------------------------------------------- Public methods (bus control) ----------------------------------------------- */

    /**
     * @brief Tries to set state of the bus to @p enabled 
     * 
     * @param enabled 
     *    if @c true method will try to enable the bus; otherwise it will try to disable it
     * 
     * @throws cifx::Error 
     *    on failure
     */
    inline void set_bus_enabled(bool enabled);

    /**
     * @returns 
     *    @retval @c true if bus is currently enabled
     *    @retval @c false otherwise
     * 
     * @throws cifx::Error 
     *    on failure
     */
    inline bool is_bus_enabled();

    /**
     * @brief Sets target state of the Master device
     * 
     * @param target_state 
     *    target state to be set
     * 
     * @throws cifx::Error 
     *    on failure
     */
    inline void set_master_target_state(ethercat::MasterState target_state);


    /**
     * @returns 
     *    structure describing current state of the EtherCAT Master device
     * 
     * @throws cifx::Error 
     *    on failure
     */
    inline ethercat::MasterStateInfo get_master_state();

    /**
     * @brief Tries to measure timing parameters of the bus
     * @returns 
     *    timing parameters on success
     * 
     * @throws cifx::Error 
     *    on failure
     */
    inline ethercat::TimingInfo get_timing_info();

public: /* ------------------------------------------------ Public methods (io) --------------------------------------------------- */

    /**
     * @brief Set the event handler routine
     * 
     * @tparam HandlerT 
     *    type of the handler
     * @param event 
     *    event to set handler for
     * @param handler 
     *    handler to be set
     */
    template<typename HandlerT>
    void set_bus_event_handler(Event event, HandlerT &&handler);

    /**
     * @brief Tries to read incoming data from the bus and dispatch it to registered slave drivers
     * 
     * @throws cifx::Error 
     *    on failure
     */
    inline void read_bus();

    /**
     * @brief Tries to write new data to the bus based on the value acquired from registered slave drivers
     * 
     * @throws cifx::Error 
     *    on failure
     */
    inline void write_bus();

private: /* ------------------------------------------------- Private types ------------------------------------------------------- */

    /**
     * @brief Pair-like structure providing single record for database of loaded slaves
     */
    struct SlaveRecord {

        /// Unique id identifying the driver
        uint64_t id;
        /// Name of the slave plugin
        std::string name;
        /// Loaded plugin
        std::shared_ptr<EthercatSlaveDriver> slave;

    };

private: /* -------------------------------------------------- Private data ------------------------------------------------------- */

    /// Reference to the ROS node (to be passed to drivers instances)
    rclcpp::Node &node;

    /// CIFX driver RAII handler
    cifx::Driver driver;
    /// CIFX device RAII handler
    cifx::Device device;
    /// CIFX communication channel used for EtherCAT communication
    cifx::Channel channel;
    /// RAII object managing host- and bus-state signalisation to the CIFX channel
    cifx::Channel::StateGuard state_guard;

    /// CIFX-specific implementation of the EtherCAT Master driver
    cifx::ethercat::Master master;
    /// Timeout for cyclical I/O operations [ms]
    std::chrono::milliseconds cyclic_io_timeout_ms;
    /// Timeout for all slaves being brought into the Operational state
    std::chrono::milliseconds slaves_up_timeout;

    /// Plugins loader
    pluginlib::ClassLoader<EthercatSlaveDriver> loader;
    /// Unique ID of the next loaded slave
    uint64_t next_unique_id { 0 };
    /// Database of loaded slave drivers
    std::vector<SlaveRecord> slaves;

};

/* ================================================================================================================================ */

} // End namespace velmwheel

/* ==================================================== Implementation includes =================================================== */

#include "velmwheel/ethercat_driver_impl/ethercat_driver_impl.hpp"

/* ================================================================================================================================ */

#endif