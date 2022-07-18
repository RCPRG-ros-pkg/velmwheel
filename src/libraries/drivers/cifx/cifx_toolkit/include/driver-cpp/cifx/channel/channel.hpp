/* ============================================================================================================================ *//**
 * @file       channel.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 4th May 2022 12:03:11 pm
 * @modified   Wednesday, 29th June 2022 11:59:23 am
 * @project    engineering-thesis
 * @brief      Definition of the RAII class wrapping description and providing related API for the 'Device's Channel' concept 
 *             of the CIFX Toolkit Framework
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __CIFX_CHANNEL_CHANNEL_H__
#define __CIFX_CHANNEL_CHANNEL_H__

/* =========================================================== Includes =========================================================== */

// Private includes
#include "cifx/common/traits.hpp"
#include "cifx/channel.hpp"
#include "cifx/device.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace cifx {

/* ========================================================= Public types ========================================================= */

template<>
struct Channel::event_traits<Channel::Event::Sync> : public std::true_type {
    using arg_type = common::traits::remove_cvref_t<typename common::traits::function_traits<Channel::SyncNotificationCallback>::arg_or_void<0>::type>;
    using callback_type = Channel::SyncNotificationCallback;
};

template<>
struct Channel::event_traits<Channel::Event::ComState> : public std::true_type {
    using arg_type = common::traits::remove_cvref_t<typename common::traits::function_traits<Channel::ComStateNotificationCallback>::arg_or_void<0>::type>;
    using callback_type = Channel::ComStateNotificationCallback;
};

/* ===================================================== Public ctors & dtors ===================================================== */

Channel::~Channel() {
    if(auto ret = xChannelClose(static_cast<Channel::Handle>(handle)); ret != CIFX_NO_ERROR)
        xTraceError("cifx::Channel::~Channel", " Failed to close an instance of the CIFX Channel (%s)", error_to_str(ret));
}

/* ================================================ Public methods (Notifications) ================================================ */

namespace details {

    /// Handy alis for notification argument type
    template<Channel::Event event> using arg_type = typename Channel::event_traits<event>::arg_type;

    template<bool flag = false>
    constexpr inline void channel_event_no_match() {
        static_assert(flag, "[cifx::Channel::notifications] Invalid event"); 
    }

    template<Channel::Event event>
    constexpr inline uint32_t event_to_cifx() {
             if constexpr( event == Channel::Event::Sync     ) return CIFX_NOTIFY_SYNC;
        else if constexpr( event == Channel::Event::ComState ) return CIFX_NOTIFY_COM_STATE;
        else channel_event_no_match();
    }

} // End namespace details


template<Channel::Event event>
void Channel::register_notification(const typename event_traits<event>::callback_type &callback) {
    
    // Register a global callback for the notification dispatch
    auto status = xChannelRegisterNotification(
        handle,
        details::event_to_cifx<event>(),
        &details::cifx_channel_callback,
        static_cast<void*>(this)
    );

    // On error throw exception
    if(status != CIFX_NO_ERROR)
        throw cifx::Error{ status, "[cifx::Channel::register_notification] Failed to register the callback" };
    
    // Cache the target callback in the object
         if constexpr( event == Event::Sync     ) callbacks.sync_notification      = callback;
    else if constexpr( event == Event::ComState ) callbacks.com_state_notification = callback;
    else details::channel_event_no_match();
}

template<Channel::Event event>
void Channel::unregister_notification() {
    
    // Uncache the target callback in the object
         if constexpr( event == Event::Sync     ) callbacks.sync_notification      = nullptr;
    else if constexpr( event == Event::ComState ) callbacks.com_state_notification = nullptr;
    else details::channel_event_no_match();
}

/* =================================================== Public methods (getters) =================================================== */

Device &Channel::get_device() {
    return device;
}


Mailbox &Channel::get_mailbox() {
    return mailbox;
}


ProcessData &Channel::get_process_data(ProcessData::Area area) {

    /**
     * @note Currently CIFX Toolchain supports only Regular Data Area ( @c 0'th )
     */
    assert(area == ProcessData::Area::Regular);

    return regular_process_data;
}

/* =========================================================== HostGuard ========================================================== */

Channel::HostGuard::HostGuard(
    Channel &channel,
    std::chrono::milliseconds timeout
) : 
    channel{ channel },
    timeout{ timeout }
{
    channel.set_host_ready(true, timeout);
}


Channel::HostGuard::~HostGuard() noexcept(false)
{
    channel.set_host_ready(false, timeout);
}

/* =========================================================== BusGuard =========================================================== */

Channel::BusGuard::BusGuard(
    Channel &channel,
    std::chrono::milliseconds timeout
) : 
    channel{ channel },
    timeout{ timeout }
{ }


Channel::BusGuard::~BusGuard() noexcept(false)
{
    channel.set_bus_on(false, timeout);
}

/* ========================================================== StateGuard ========================================================== */

Channel::StateGuard::StateGuard(
    Channel &channel,
    std::chrono::milliseconds bus_timeout,
    std::chrono::milliseconds host_timeout
) : 
    channel{ channel },
    bus_timeout{ bus_timeout },
    host_timeout{ host_timeout }
{
    channel.set_host_ready(true, host_timeout);
}


Channel::StateGuard::~StateGuard() noexcept(false)
{
    channel.set_bus_on(false, bus_timeout);
    channel.set_host_ready(false, host_timeout);
}

/* ================================================================================================================================ */

} // End namespace cifx

/* ================================================================================================================================ */

#endif
