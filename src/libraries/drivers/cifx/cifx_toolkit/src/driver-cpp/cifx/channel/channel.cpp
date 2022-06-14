/* ============================================================================================================================ *//**
 * @file       channel.cpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 4th May 2022 6:58:27 pm
 * @modified   Wednesday, 25th May 2022 9:45:25 pm
 * @project    engineering-thesis
 * @brief      Definition of the RAII class wrapping description and providing related API for the 'Device's Channel' concept 
 *             of the CIFX Toolkit Framework
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

/* =========================================================== Includes =========================================================== */

// Private includes
#include "cifx/common/traits.hpp"
#include "cifx/channel.hpp"
#include "cifx/device.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace cifx {
    
/* ===================================================== Public ctors & dtors ===================================================== */

Channel::Channel(Device &device, uint32_t channel_id) :
    device{ device },
    mailbox{ *this },
    regular_process_data{ *this, ProcessData::Area::Regular }
{
    
    // Open the channels's handle; if routine failed, throw error
    if(auto ret = xChannelOpen(
        static_cast<Driver::Handle>(device.driver),
        const_cast<char *>(device.name.c_str()),
        channel_id,
        static_cast<Channel::Handle*>(&handle)
    ); ret != CIFX_NO_ERROR) {

        std::stringstream message;
        
        // Create the error message
        message << "Failed to open a new instance of the CIFX Channel";
        // Specify errounous driver
        message << " ([device]: '" << device.name.c_str() << "', [channel]: " << channel_id << "')";

        throw Error { ret, "Channel::Channel", message.str() };
    }
    
}

/* ================================================ Public methods (Notifications) ================================================ */

namespace details {

    void cifx_channel_callback(
        uint32_t notification_event,
        [[maybe_unused]] uint32_t data_len,
        void* data,
        void* context
    ) {

        // Get reference to the @ref Channel object
        Channel &channel = *static_cast<Channel*>(context);

        // Dispatch callback
        switch(notification_event) {
                
            // Dispatch handler for the 'Sync' event
            case CIFX_NOTIFY_SYNC:

                if(channel.callbacks.sync_notification)
                    channel.callbacks.sync_notification();
            
                break;
                
            // Dispatch handler for the 'Communication state' event
            case CIFX_NOTIFY_COM_STATE:

                // Check if non-null data is given (as exppected)
                if(data == nullptr)
                    return;

                // Call handler
                if(channel.callbacks.com_state_notification)
                    channel.callbacks.com_state_notification(
                        *static_cast<typename Channel::event_traits<Channel::Event::ComState>::arg_type*>(data));
            
                break;
                
            // Non-supported event
            default: break;
        }

        return;
    }
    
} // End namespace details

/* =============================================== Public methods (Firmware control) ============================================== */


void Channel::set_host_ready(bool ready, std::chrono::milliseconds timeout_ms) {

    uint32_t state;

    // Get master state known by the device
    auto status = xChannelHostState(
        handle,
        ready ? CIFX_HOST_STATE_READY : CIFX_HOST_STATE_NOT_READY,
        &state,
        static_cast<uint32_t>(timeout_ms.count())
    );

    // On error throw exception
    if(status != CIFX_NO_ERROR)
        throw cifx::Error{ status, "[cifx::Channel::set_host_ready] Failed to communicate with the device" };
}


bool Channel::is_host_ready(std::chrono::milliseconds timeout_ms) {

    uint32_t state;

    // Get master state known by the device
    auto status = xChannelHostState(
        handle,
        CIFX_HOST_STATE_READ,
        &state,
        static_cast<uint32_t>(timeout_ms.count())
    );

    // On error throw exception
    if(status != CIFX_NO_ERROR)
        throw cifx::Error{ status, "[cifx::ethercat::Master::is_master_ready] Failed to communicate with the device" };

    return (state == CIFX_HOST_STATE_READ);
}


void Channel::set_bus_on(bool ready, std::chrono::milliseconds timeout_ms) {

    uint32_t state;

    // Get bus state
    auto status = xChannelBusState(
        handle,
        ready ? CIFX_BUS_STATE_ON : CIFX_BUS_STATE_OFF,
        &state,
        static_cast<uint32_t>(timeout_ms.count())
    );

    // On error throw exception
    if(status != CIFX_NO_ERROR)
        throw cifx::Error{ status, "[cifx::Channel::set_bus_on] Failed to communicate with the device" };
}


bool Channel::is_bus_on(std::chrono::milliseconds timeout_ms) {

    uint32_t state;

    // Get bus state
    auto status = xChannelBusState(
        handle,
        CIFX_BUS_STATE_GETSTATE,
        &state,
        static_cast<uint32_t>(timeout_ms.count())
    );

    // On error throw exception
    if(status != CIFX_NO_ERROR)
        throw cifx::Error{ status, "[cifx::ethercat::Master::is_bus_on] Failed to communicate with the device" };

    return (state == CIFX_BUS_STATE_ON);
}

/* ================================================================================================================================ */

} // End namespace cifx

