/* ============================================================================================================================ *//**
 * @file       process_data.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 4th May 2022 12:03:11 pm
 * @modified   Tuesday, 28th June 2022 2:12:47 pm
 * @project    engineering-thesis
 * @brief      Definition of the class wrapping description and providing related API for the 'Process Data' concept of the CIFX 
 *             Toolkit Framework
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __CIFX_CHANNEL_PROCESS_DATA_PROCESS_DATA_H__
#define __CIFX_CHANNEL_PROCESS_DATA_PROCESS_DATA_H__

/* =========================================================== Includes =========================================================== */

// Private includes
#include "cifx/channel/process_data.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace cifx {

/* ================================================ Public methods (Notifications) ================================================ */

/**
 * @addtogroup private
 * @{
 */

namespace details {

    template<bool flag = false>
    constexpr inline void process_data_event_no_match() {
        static_assert(flag, "[cifx::ProcessData::notifications] Invalid event"); 
    }

    template<ProcessData::Event event>
    constexpr inline uint32_t event_to_cifx(ProcessData::Area area) {
             if constexpr( event == ProcessData::Event::Input  ) return area == ProcessData::Area::Regular ? CIFX_NOTIFY_PD0_IN  : CIFX_NOTIFY_PD1_IN;
        else if constexpr( event == ProcessData::Event::Output ) return area == ProcessData::Area::Regular ? CIFX_NOTIFY_PD0_OUT : CIFX_NOTIFY_PD1_OUT;
        else process_data_event_no_match();
    }

} // End namespace details

/**
 * @}
 */


template<ProcessData::Event event>
void ProcessData::register_notification(const std::function<void(void)> &callback) {
    
    // Register a global callback for the notification dispatch
    auto status = xChannelRegisterNotification(
        channel.handle,
        details::event_to_cifx<event>(area),
        &details::cifx_process_data_callback,
        static_cast<void*>(this)
    );

    // On error throw exception
    if(status != CIFX_NO_ERROR)
        throw cifx::Error{ status, "[cifx::ProcessData::register_notification] Failed to register the callback" };
    
    // Cache the target callback in the object
         if constexpr( event == Event::Input  ) callbacks.input  = callback;
    else if constexpr( event == Event::Output ) callbacks.output = callback;
    else details::process_data_event_no_match();
}

template<ProcessData::Event event>
void ProcessData::unregister_notification() {
    
    // Uncache the target callback in the object
         if constexpr( event == Event::Input  ) callbacks.input  = nullptr;
    else if constexpr( event == Event::Output ) callbacks.output = nullptr;
    else details::process_data_event_no_match();
}


/* =============================================== Public mehotds (Synchronous I/O) =============================================== */

void ProcessData::read(
    std::size_t offset,
    ranges::span<uint8_t> buffer,
    std::chrono::milliseconds timeout_ms
) {

    // Read process data
    if(auto ret = xChannelIORead(
        channel.handle,
        static_cast<uint32_t>(area),
        offset,
        buffer.size(),
        static_cast<void*>(buffer.data()),
        static_cast<uint32_t>(timeout_ms.count())
    ); ret != CIFX_NO_ERROR)
        throw cifx::Error{ ret, "cifx::ProcessData::read", "Failed to read process data" };
}


void ProcessData::write(
    std::size_t offset,
    ranges::span<const uint8_t> buffer,
    std::chrono::milliseconds timeout_ms
) {

    // Write process data
    if(auto ret = xChannelIOWrite(
        channel.handle,
        static_cast<uint32_t>(area),
        offset,
        buffer.size(),
        const_cast<void*>(static_cast<const void*>(buffer.data())),
        static_cast<uint32_t>(timeout_ms.count())
    ); ret != CIFX_NO_ERROR)
        throw cifx::Error{ ret, "cifx::ProcessData::write", "Failed to write process data" };
}

/* ==================================================== Protected ctors & dtors =================================================== */

ProcessData::ProcessData(Channel &channel, Area area) :
    channel{ channel },
    area{ area }
{ }

/* ================================================================================================================================ */

} // End namespace cifx

/* ================================================================================================================================ */

#endif
