/* ============================================================================================================================ *//**
 * @file       driver.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 20th April 2022 8:06:44 pm
 * @modified   Wednesday, 27th April 2022 3:50:20 pm
 * @project    engineering-thesis
 * @brief      Definitions of public driver configuration methods of the the EtherCAT Master class template
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __CIFX_ETHERCAT_MASTER_IMPL_DRIVER_H__
#define __CIFX_ETHERCAT_MASTER_IMPL_DRIVER_H__

/* =========================================================== Includes =========================================================== */

// System includes
#include <cstdint>
#include <mutex>
// Boost includes
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
// CIFX includes
#include "EcmIF_Public.h"
// Private includes
#include "cifx/ethercat/master.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace cifx {
namespace ethercat {

/* ============================================= Public methods (driver configuration) ============================================ */

template<typename Lock>
void Master<Lock>::set_timeout(TimeoutAction action, std::chrono::milliseconds timeout) {

    std::lock_guard<Lock> glock(lock);

    // Check if vali action action identifier has been given
    if(to_underlying(action) >= to_underlying(TimeoutAction::Num))
        throw cifx::Error{ CIFX_INVALID_PARAMETER, "[cifx::ethercat::Master::set_timeout] Invalid action ID" };

    // Set the timeout
    timeouts[to_underlying(action)] = timeout;
}


template<typename Lock>
std::chrono::milliseconds Master<Lock>::get_timeout(TimeoutAction action) const {

    std::lock_guard<Lock> glock(lock);

    // Check if vali action action identifier has been given
    if(to_underlying(action) >= to_underlying(TimeoutAction::Num))
        throw cifx::Error{ CIFX_INVALID_PARAMETER, "[cifx::ethercat::Master::get_timeout] Invalid action ID" };

    return timeouts[to_underlying(action)];
}


template<typename Lock>
template<notifications::Event event>
void Master<Lock>::register_notification_handler(const notifications::callback_type<event> &handler) {

    // Check if vali action action identifier has been given
    static_assert(to_underlying(event) < to_underlying(notifications::Event::Num),
        "[cifx::ethercat::Master::register_notification_handler] Invalid event ID"
    );

    std::lock_guard<Lock> glock(lock);
    
    // Register a global callback for the notification dispatch
    auto status = xChannelRegisterNotification(
        cifx_channel,
        notifications::event_to_cifx<event>(),
        &notifications::cifx_callback<Lock>,
        static_cast<void*>(this)
    );

    // On error throw exception
    if(status != CIFX_NO_ERROR)
        throw cifx::Error{ status, "[cifx::ethercat::Master::register_notification_handler] Failed to register the callback" };
    
    // Cache the target callback in the object
    notifications::event_to_callback<event>(callbacks) = handler;
}


template<typename Lock>
template<typename notifications::Event event>
void Master<Lock>::unregister_notification_handler() {

    // Check if vali action action identifier has been given
    static_assert(to_underlying(event) < to_underlying(notifications::Event::Num),
        "[cifx::ethercat::Master::register_notification_handler] Invalid event ID"
    );

    std::lock_guard<Lock> glock(lock);
    
    // Unregister a global callback from the notification dispatch
    auto status = xChannelUnregisterNotification(
        cifx_channel,
        notifications::event_to_cifx<event>()
    );

    // On error throw exception
    if(status != CIFX_NO_ERROR)
        throw cifx::Error{ status, "[cifx::ethercat::Master::unregister_notification_handler] Failed to unregister the callback" };
    
    // Drop locally cached callback
    notifications::event_to_callback<event>(callbacks) = nullptr;
}

/* ================================================================================================================================ */

} // End namespace ethercat
} // End namespace cifx

#endif

