/* ============================================================================================================================ *//**
 * @file       channel.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 4th May 2022 12:10:47 pm
 * @modified   Wednesday, 25th May 2022 9:38:33 pm
 * @project    engineering-thesis
 * @brief      Definition of the RAII class wrapping description and providing related API for the 'Device's Channel' concept of the CIFX 
 *             Toolkit Framework
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __CIFX_CHANNEL_H__
#define __CIFX_CHANNEL_H__

/* =========================================================== Includes =========================================================== */

// System includes
#include <mutex>
#include <optional>
// External includes
#include "range/v3/span.hpp"
// Private includes
#include "cifx/config.hpp"
#include "cifx/error.hpp"
#include "cifx/driver.hpp"
#include "cifx/device.hpp"
#include "cifx/channel/mailbox.hpp"
#include "cifx/channel/process_data.hpp"
// CIFX includes
#include "cifxDriver.h"
#include "cifXHWFunctions.h"

/* ========================================================== Namespaces ========================================================== */

namespace cifx {

/* ========================================================= Declarations ========================================================= */

namespace details {
    
    void cifx_channel_callback(
        uint32_t notification_event,
        uint32_t data_len,
        void* data,
        void* context
    );

} // End namespace details

/* ============================================================= Class ============================================================ */

// Forward declare the device class
class Device;

/**
 * @brief RAII class wrapping description and providing related API for the 'Device's Channel' concept 
 *    of the CIFX Toolkit Framework
 */
class Channel {

    /// Make Mailbox class a friend to let it access channel handle
    friend class Mailbox;
    /// Make ProcessData class a friend to let it access channel handle
    friend class ProcessData;

public: /* ------------------------------------------------------ Public types ---------------------------------------------------- */

    /// Local typename for the CIFX Driver's Channel handle
    using Handle = CIFXHANDLE;

    /**
     * @brief Events related to the Channel's Mailbox that the notification can be 
     *    registered for
     * @see 'cifx_toolkit/doc/cifX API PR 09 EN.pdf'
     */
    enum class Event {
        Sync,
        ComState
    };
    
    /// Type of the callback associated with the @c Event::Sync callback
    using SyncNotificationCallback = std::function<void(void)>;
    /// Type of the callback associated with the @c Event::ComState callback
    using ComStateNotificationCallback = std::function<void(const CIFX_NOTIFY_COM_STATE_T&)>;

    /// Traits of the notification event ( used to define argument type for @ref register_notification(...) )
    template<Event event> struct event_traits : public std::false_type { };

public: /* ---------------------------------------------------- Public constructors ----------------------------------------------- */

    /**
     * @brief Constructs a new object wrapping description and providing related API 
     *     for the 'Device's Channel' concept of the CIFX Toolkit Framework
     * 
     * @param driver 
     *    handle to the CIFX driver entry-point to be used to open the channel
     * @param channel_id 
     *    index of the @p device's channel to be opened
     * 
     * @throws Error
     *    when the procedure failed to open the channel handle in the toolkit
     */
    Channel(Device &device, uint32_t channel_id);

    /**
     * @brief Destroys the Channel object closing related communication channel
     */
    inline ~Channel();

public: /* -------------------------------------------- Public methods (Notifications) -------------------------------------------- */

    /**
     * @brief Registers callback fot eh given @p event
     * 
     * @tparam event 
     *    event to register callback for
     * @param callback 
     *    target callback
     */
    template<Event event>
    void register_notification(const typename event_traits<event>::callback_type &callback);

    /**
     * @brief Unregisters callback fot eh given @p event
     * 
     * @tparam event 
     *    event to unregister callback for
     */
    template<Event event>
    void unregister_notification();

public: /* ------------------------------------------- Public methods (Firmware control) ------------------------------------------ */

    /**
     * @brief Sets current state of the Host driver in the CIFX device
     * 
     * @param ready 
     *    state of the Host driver
     * @param timeout_ms
     *    timeout for the action
     * 
     * @throws cifx::Error 
     *    on error
     */
    inline void set_host_ready(bool ready, std::chrono::milliseconds timeout_ms);

    /**
     * @brief Checks current state of the Host driver in the CIFX device
     * 
     * @param timeout_ms
     *    timeout for the action
     * @returns 
     *    @retval @c true if Host driver is configured as ready
     *    @retval @c false otherwise
     * 
     * @throws cifx::Error 
     *    on error
     */
    inline bool is_host_ready(std::chrono::milliseconds timeout_ms);

    /**
     * @brief Sets current state of the fieldbus in the CIFX device
     * 
     * @param ready 
     *    state of the fieldbus bus to be set
     * @param timeout_ms
     *    timeout for the action
     * 
     * @throws cifx::Error 
     *    on error
     */
    inline void set_bus_on(bool ready, std::chrono::milliseconds timeout_ms);

    /**
     * @brief Checks current state of the fieldbus bus in the CIFX device
     * 
     * @param timeout_ms
     *    timeout for the action
     * @returns 
     *    @retval @c true if fieldbus bus is configured as ready
     *    @retval @c false otherwise
     * 
     * @throws cifx::Error 
     *    on error
     */
    inline bool is_bus_on(std::chrono::milliseconds timeout_ms);

public: /* ------------------------------------------------ Public methods (getters) ---------------------------------------------- */

    /**
     * @returns 
     *    reference to the associated device
     */
    inline Device &get_device();

    /**
     * @returns 
     *    reference to the channel's mailbox
     */
    inline Mailbox &get_mailbox();

    /**
     * @param area 
     *    area of the process data to be returned (CIFX toolkit supports only Regular area at the moment)
     * @returns 
     *    reference to the channel's process data
     */
    inline ProcessData &get_process_data(ProcessData::Area area);

private: /* ---------------------------------------------------- Private friends -------------------------------------------------- */
    
    // /Make CIFX-specific interface notification callback function a friend to let it access notifications
    friend void details::cifx_channel_callback(
        uint32_t notification_event,
        uint32_t data_len,
        void* data,
        void* context
    );
        
private: /* ----------------------------------------------- Private member variables ---------------------------------------------- */

    /// Reference to the associated device
    Device &device;

    /// Handle to the wrapped channel
    Handle handle;

    /// Notification callbacks
    struct {

        /// Callback registered for @c Event::Sync event
        SyncNotificationCallback sync_notification;
        /// Callback registered for @c Event::ComState event
        ComStateNotificationCallback com_state_notification;

    } callbacks;

    /// Channel's mailbox
    Mailbox mailbox;

    /// Channel's regular process data
    ProcessData regular_process_data;

};

/* ================================================================================================================================ */

} // End namespace cifx

/* ==================================================== Implementation includes =================================================== */

// Private includes
#include "cifx/channel/channel.hpp"
#include "cifx/channel/mailbox/mailbox.hpp"
#include "cifx/channel/process_data/process_data.hpp"

/* ================================================================================================================================ */

#endif
