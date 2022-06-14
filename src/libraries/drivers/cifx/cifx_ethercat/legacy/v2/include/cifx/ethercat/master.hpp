/* ============================================================================================================================ *//**
 * @file       master.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Monday, 14th February 2022 2:33:20 pm
 * @modified   Thursday, 28th April 2022 11:23:03 am
 * @project    engineering-thesis
 * @brief      Declaration of the EtherCAT Master class managing access to the EtherCAT bus over the CIFX driver
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __CIFX_ETHERCAT_MASTER_H__
#define __CIFX_ETHERCAT_MASTER_H__

/* =========================================================== Includes =========================================================== */

// System includes
#include <functional>
#include <vector>
#include <memory>
#include <chrono>
#include <string_view>
// CIFX includes
#include "cifx/toolkit.hpp"
// Private includes
#include "cifx/ethercat/common/utilities.hpp"
#include "cifx/ethercat/common/locks.hpp"
#include "cifx/ethercat/common/pdo.hpp"
#include "cifx/ethercat/master/notfications.hpp"
#include "cifx/ethercat/master/timing.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace cifx {
namespace ethercat {

/* ============================================================ Classes =========================================================== */

// Forward declare Slave class template
class Slave;

/**
 * @class Master
 * @brief Class providing a C++ interface for interacting with CIFX interface device
 *    running EtherCAT protocol firmware
 * @details Master class provides a simplified interface around EtherCAT stack API
 *    provided by the CIFX toolkit for interacting with CIFX interface device running 
 *    EtherCAT protocol firmware. The class has been designed based on the
 *    @b velmwheel_ec_driver by Adam Kowalewski and provides similar capabilities in 
 *    a more flexible, modern-cpp-style format. Morover it implements interface for 
 *    utilizing externally-implemented interfaces for Slave devices present on the 
 *    EtherCAT bus.
 * 
 * @tparam Lock 
 *    type used to synchronsie classes' methods; has to implement lock() and unlock()
 *    methods
 * 
 * @note At the moment relation between @ref Master and @ref Slave class is implemented in
 *    a client-server manner. At each IO read/write master class requests registered slaves
 *    to update their input/output PDOs. Taking into account that slaves are implemented
 *    in terms of virtual interface it may lead to unnececerely large dynamicly dispatched
 *    methods calls in situations when the implementation of the Slave driver in fact does
 *    not require to obtain/write down new value of the input/output PDO. This implementation
 *    has been introduced to avoid mutual calls of synchronised methods between Master and
 *    Slave class which migh lead to (depending on implementation):
 * 
 *       - dead lock conditions when the Slave tries to update output PDO in the Master
 *         and Master tries to update input PDO in the Slave
 *       - unnecesary long waiting times in the Slave trying to obtain value of the input
 *         PDO while Master wait on IO operation
 * 
 *    These problems may be eliminated without usage of the polling implementation if a proper
 *    synchronisation protocol between Master and Slave classes is developed. Current implementation
 *    is just simple, single-lock-based solution that may or may not turn out to be enough
 *    efficient in the real world application depending on number of registered slaves, bus
 *    timing configuration and computational resources.
 * @todo Implement data exchange interface between @ref Master and @ref Slave class that would
 *    not require polling registered slaves at each I/O operation. Provide proper synchronisation
 *    mechanism (probably concerning individual locks for @ref Master class synchronisation and
 *    Process Data Image buffers' content synchronisation) 
 * 
 * @note Current implementation provides byte-alligned mapping between PDOs and Process Data Image.
 *    In general allows bit-aligned PDOs. Implementing such a capability would require smart bit-to-bit
 *    copying algorithm to avoid unnesesary single-bit-access oepration. In the current use cases
 *    PDOs of all devices on utilized busses have their PDOs byte aligned and so fine-grained 
 *    acess is not required.
 * @todo Implement bit-aligned PDOs mapping
 */
template<typename Lock = locks::EmptyLock>
class Master {

public: /* ------------------------------------------------- Public ctors & dtors ------------------------------------------------- */

    /**
     * @brief Construct a new Master interface object. Creates a new, local connection to the CIFX @p toolkit 
     *    and opens @p channel_id channel of the CIFX deviced named @p device_name that has been already 
     *    registered to the @p toolchain . The opened channel is used for EtherCAT communitaction
     * 
     * @param toolkit 
     *    reference to the CIFX toolkit manager to be used by the object to find the device named @p device_name 
     * @param device_name 
     *    name of the CIFX device to be used for communication
     * @param channel_id 
     *    ID of the channel of the CIFX device named @p device_name to be used for communication
     * 
     * @throws cifx::Error 
     *    on error
     * 
     * @note Method assumes that the ENI configuration file configured for the EtherCAT communication has the
     *    same index as @p channel_id in the registered device 
     */
    Master(
        cifx::Toolkit &toolkit,
        std::string_view device_name,
        uint32_t channel_id
    );

    /**
     * @brief Destroy the Master object deinitializing the EtherCAT communication channel
     *    and closing local connection to the CIFX driver
     */
    ~Master();

public: /* ---------------------------------------- Public methods (driver configuration) ----------------------------------------- */

    /**
     * @brief Sets timeout associated with the given @p action 
     * 
     * @param action 
     *    target action
     * @param timeout 
     *    timeout to be set
     * 
     * @throws cifx::Error 
     *    on error
     * 
     * @synchronised
     */
    void set_timeout(TimeoutAction action, std::chrono::milliseconds timeout);

    /**
     * @param action 
     *    target action
     * @returns 
     *    current timeout of the 'put packet' actions for the CIFX interface
     * 
     * @throws cifx::Error 
     *    on error
     * 
     * @synchronised
     */
    std::chrono::milliseconds get_timeout(TimeoutAction action) const;

    /**
     * @brief Registers notification handler for the given @tparam event
     * 
     * @tparam event 
     *    event to register handler for
     * @param handler 
     *    handler to be registered
     * 
     * @throws cifx::Error 
     *    on error
     * 
     * @synchronised
     */
    template<notifications::Event event>
    void register_notification_handler(const notifications::callback_type<event> &handler);

    /**
     * @brief Unregisters notification handler for the given @tparam event
     * 
     * @tparam event 
     *    event to unregister handler for
     * 
     * @throws cifx::Error 
     *    on error
     * 
     * @synchronised
     */
    template<notifications::Event event>
    void unregister_notification_handler();
    
public: /* ----------------------------------------- Public methods (device configuration) ---------------------------------------- */
    
    /**
     * @returns 
     *    path to the ENI file used to configure CIFX EtherCAT stack
     */
    inline const std::string &get_eni_path() const noexcept;

    /**
     * @returns 
     *    preconfigured period of the EtherCAT bus
     */
    inline const std::chrono::nanoseconds &get_bus_cycle_time() const noexcept;

    /**
     * @brief Sets current state of the Master driver in the CIFX device
     * 
     * @param ready 
     *    state of the Master driver
     * 
     * @throws cifx::Error 
     *    on error
     * 
     * @synchronised
     */
    inline void set_master_ready(bool ready);

    /**
     * @brief Checks current state of the Master driver in the CIFX device
     * 
     * @returns 
     *    @retval @c true if master driver is configured as ready
     *    @retval @c false otherwise
     * 
     * @throws cifx::Error 
     *    on error
     * 
     * @synchronised
     */
    bool is_master_ready();

    /**
     * @brief Sets current state of the EtherCAT bus in the CIFX device
     * 
     * @param ready 
     *    state of the EtherCAT bus to be set
     * 
     * @throws cifx::Error 
     *    on error
     * 
     * @synchronised
     */
    inline void set_bus_on(bool ready);

    /**
     * @brief Checks current state of the EtherCAT bus in the CIFX device
     * 
     * @returns 
     *    @retval @c true if EtherCAT bus is configured as ready
     *    @retval @c false otherwise
     * 
     * @throws cifx::Error 
     *    on error
     * 
     * @synchronised
     */
    bool is_bus_on();

    /**
     * @brief Configures synchronisation mode for the EtherCAT communication channel of the CIFX device
     * 
     * @param mode 
     *    mode to be configured
     * 
     * @throws cifx::Error 
     *    on error
     * 
     * @synchronised
     */
    void configure_sync_mode(SyncMode mode);

    /**
     * @brief Reads timing informations form the device
     * 
     * @returns 
     *    current timing information concerning the EtherCAT bus
     * 
     * @throws cifx::Error 
     *    on error
     * 
     * @synchronised
     */
    TimingInfo get_timing_info();

public: /* -------------------------------------------- Public methods (IO operations) -------------------------------------------- */

    /**
     * @brief Tries to read Input Process Image from the CIFX device. On success updates notifies
     *    all registered slave drivers to update based on the incoming Input Process Image
     * 
     * @throws Error 
     *    if read operation failed
     * 
     * @note This method is wrapper around @ref xChannelIORead(...) function. It's always
     *    called for @c 0'th <i>Input Data Area</i> ('regular' area, opposed to 'priority' one).
     * 
     * @synchronised
     */
    void read_io();

    /**
     * @brief Tries to write Output Process Image to the CIFX device. Before writting notifies all
     *    registered slave drivers to update associated PDOs in the Output Process Image
     * 
     * @throws Error 
     *    if read operation failed
     * 
     * @note This method is wrapper around @ref xChannelIOWrite(...) function. It's always
     *    called for @c 0'th <i>Output Data Area</i> ('regular' area, opposed to 'priority' one).
     * 
     * @synchronised
     */
    void write_io();

    /**
     * @returns 
     *    copy of the current Input Process Data buffer
     * 
     * @note This method is aimed for debugging purposes and normally should not be used
     *    in the client code
     * 
     * @synchronised
     */
    std::vector<uint8_t> get_input_process_image() const noexcept;
    
    /**
     * @returns 
     *    copy of the current Output Process Data buffer
     * 
     * @note This method is aimed for debugging purposes and normally should not be used
     *    in the client code
     * 
     * @synchronised
     */
    std::vector<uint8_t> get_output_process_image() const noexcept;

public: /* -------------------------------------------- Public methods (slave drivers) -------------------------------------------- */

    /**
     * @brief Registers @p slave interface to the Master
     * 
     * @param slave 
     *    reference to the slave interface to be registered
     * 
     * @throws cifx::Error
     *    if @p slave is empty
     * 
     * @note If the @p slave is already registerd, method returns immediatelly
     * 
     * @synchronised
     */
    void register_slave(std::shared_ptr<Slave> slave);

    /**
     * @brief Unregisters @p slave interface from the Master
     * 
     * @param slave 
     *    pointer to the slave interface to be unregistered
     * 
     * @throws cifx::Error
     *    if @p slave is empty
     * @throws std::invalid_argument 
     *    if @p slave has not been registered to the object
     * 
     * @synchronised
     */
    inline void unregister_slave(std::shared_ptr<Slave> slave);

protected: /* ------------------------------------------------ Protected methods -------------------------------------------------- */

    /// @brief Non-synchronise implementation of the @ref set_master_ready() method
    void set_master_ready_impl(bool ready);
    /// @brief Non-synchronise implementation of the @ref set_bus_on() method
    void set_bus_on_impl(bool ready);

    /// @brief Non-synchronise implementation of the @ref unregister_slave() method
    void unregister_slave_impl(std::shared_ptr<Slave> slave);

private: /* --------------------------------------------------- Private friends --------------------------------------------------- */

    /// Make @ref cifx_notification_callback(...) a friend to let it access callback functors
    friend void notifications::cifx_callback<Lock>(
        uint32_t notification_event,
        uint32_t data_len,
        void* data,
        void* context
    );

private: /* ------------------------------------------------ Private data members ------------------------------------------------- */

    // Lock synchronising accessd to concurency-sensitive methods
    mutable Lock lock;

private: /* ----------------------------------------- Private data members (CIFX driver) ------------------------------------------ */

    // Handle to the CIFX driver connection
    Toolkit::DriverHandle const cifx_driver;
    // Handle to the CIFX channel utilized for EtherCAT communication
    Toolkit::ChannelHandle const cifx_channel;

    // Path to the ENI file loaded to the device
    const std::string eni_path;
    // Preconfigured bus cycle
    const std::chrono::nanoseconds bus_cycle;
    
    // Timeout of the I/O operations
    std::array<std::chrono::milliseconds, to_underlying(TimeoutAction::Num)> timeouts {
        /* PutPacket timeout */ std::chrono::milliseconds{  100 },
        /* GetPacket timeout */ std::chrono::milliseconds{ 5000 },
        /* ReadIO timeout    */ std::chrono::milliseconds{ 1000 },
        /* WriteIO timeout   */ std::chrono::milliseconds{ 1000 }
    };
    
    // Callback handlers for the CIFX events
    notifications::Callbacks callbacks;

private: /* ------------------------------------------- Private data members (Drivers) -------------------------------------------- */

    // Input Process Image (cyclical readings from bus devices)
    std::vector<uint8_t> input_process_image;
    // Output Process Image (cyclical writes to bus devices)
    std::vector<uint8_t> output_process_image;

    // Slave interfaces registered to the master
    std::map<std::shared_ptr<Slave>, pdo::ReferencesSet> slaves;

};

/* ================================================================================================================================ */

} // End namespace ethercat
} // End namespace cifx

/* ==================================================== Implementation includes =================================================== */

#include "cifx/ethercat/master/impl/master.hpp"
#include "cifx/ethercat/master/impl/driver.hpp"
#include "cifx/ethercat/master/impl/device.hpp"
#include "cifx/ethercat/master/impl/io.hpp"
#include "cifx/ethercat/master/impl/slaves.hpp"

/* ================================================================================================================================ */

#endif
