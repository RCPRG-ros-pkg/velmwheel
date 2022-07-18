/* ============================================================================================================================ *//**
 * @file       mailbox.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 4th May 2022 12:03:11 pm
 * @modified   Thursday, 30th June 2022 2:21:51 pm
 * @project    engineering-thesis
 * @brief      Definition of the class wrapping description and providing related API for the 'Mailbox' concept of the CIFX 
 *             Toolkit Framework
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __CIFX_CHANNEL_MAILBOX_MAILBOX_H__
#define __CIFX_CHANNEL_MAILBOX_MAILBOX_H__

/* =========================================================== Includes =========================================================== */

// CIFX includes
#include "Hil_Results.h"
// Private includes
#include "cifx/common/traits.hpp"
#include "cifx/channel/mailbox.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace cifx {

/* ========================================================= Public types ========================================================= */

template<>
struct Mailbox::event_traits<Mailbox::Event::RxMailboxFull> : public std::true_type {
    using arg_type = common::traits::remove_cvref_t<
        typename common::traits::function_traits<
            Mailbox::RxMailboxFullCallback
        >::arg_or_void<0>::type
    >;
    using callback_type = Mailbox::RxMailboxFullCallback;
};

template<>
struct Mailbox::event_traits<Mailbox::Event::TxMailboxEmpty> : public std::true_type {
    using arg_type = common::traits::remove_cvref_t<
        typename common::traits::function_traits<
            Mailbox::TxMailboxEmptyCallback
        >::arg_or_void<0>::type
    >;
    using callback_type = Mailbox::TxMailboxEmptyCallback;
};

/* =============================================== Public methods (Asynchronous I/O) ============================================== */

template<typename PacketT>
void Mailbox::put_packet(const PacketT &packet, std::chrono::milliseconds timeout_ms) {
    return put_packet_impl(to_packet(packet), timeout_ms);
}


template<typename PacketT>
void Mailbox::get_packet(PacketT &packet, std::chrono::milliseconds timeout_ms) {
    return get_packet_impl(to_packet(packet), std::size(packet), timeout_ms);
}


template<typename RequestT, typename ResponseT>
void Mailbox::exchange_packet(
    const RequestT &request,
    ResponseT &response,
    std::chrono::milliseconds timeout_ms
) {
    return exchange_packet_impl(
        to_packet(request),
        to_packet(response),
        sizeof(ResponseT),
        timeout_ms
    );
}


template<typename RequestT, typename ResponseT>
void Mailbox::exchange_packet(
    const RequestT &request,
    ResponseT &response,
    std::chrono::milliseconds request_timeout_ms,
    std::chrono::milliseconds response_timeout_ms
) {
    return exchange_packet_impl(
        to_packet(request),
        to_packet(response),
        sizeof(ResponseT),
        request_timeout_ms,
        response_timeout_ms
    );
}

template<typename RequestT, typename ResponseT>
void Mailbox::exchange_packet_with_timeout_update(
    const RequestT &request,
    ResponseT &response,
    std::chrono::milliseconds &timeout_ms
) {
    constexpr auto context = "[cifx::Mailbox::exchange_packet_with_timeout_update]";

    // Denote start time of the routine
    auto start_time_point = std::chrono::steady_clock::now();

    // Exchange packet with the CIFX device
    exchange_packet(request, response, timeout_ms);
    
    // Check if transfer aborted
    if(response.tHead.ulSta != SUCCESS_HIL_OK) {
        std::stringstream ss;
        ss << "CIFX device reported error [ulSta: 0x" << std::hex << std::uppercase << response.tHead.ulSta << "]";
        throw cifx::Error{ CIFX_DEV_FUNCTION_FAILED, context, ss.str() };
    }

    using namespace std::literals::chrono_literals;

    // Update timeout left
    timeout_ms -= std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time_point);
    // If timeout reached, throw error
    if(timeout_ms <= 0ms)
        throw cifx::Error{ CIFX_DEV_EXCHANGE_TIMEOUT, context, "Exchange timeouted" };
}


template<typename RequestT, typename ResponseT>
void Mailbox::exchange_partial_packet(
    RequestT &request,
    ResponseT &response,
    std::chrono::milliseconds &timeout_ms
) {
    // Exchange packet
    exchange_packet_with_timeout_update(
        request,
        response,
        timeout_ms
    );

    // Set packet's destination ID for the next request the one returned by the CIFX device in the response
    request.tHead.ulDestId = response.tHead.ulDestId;
}

/* ================================================ Public methods (Notifications) ================================================ */

namespace details {

    template<bool flag = false>
    constexpr inline void mailbox_event_no_match() {
        static_assert(flag, "[cifx::Mailbox::notifications] Invalid event"); 
    }

    template<Mailbox::Event event>
    constexpr inline uint32_t event_to_cifx() {
             if constexpr( event == Mailbox::Event::RxMailboxFull  ) return CIFX_NOTIFY_RX_MBX_FULL;
        else if constexpr( event == Mailbox::Event::TxMailboxEmpty ) return CIFX_NOTIFY_TX_MBX_EMPTY;
        else mailbox_event_no_match();
    }

} // End namespace details

template<Mailbox::Event event>
void Mailbox::register_notification(const typename event_traits<event>::callback_type &callback) {
    
    // Register a global callback for the notification dispatch
    auto status = xChannelRegisterNotification(
        channel.handle,
        details::event_to_cifx<event>(),
        &details::cifx_mailbox_callback,
        static_cast<void*>(this)
    );

    // On error throw exception
    if(status != CIFX_NO_ERROR)
        throw cifx::Error{ status, "[cifx::Mailbox::register_notification] Failed to register the callback" };
    
    // Cache the target callback in the object
         if constexpr( event == Event::RxMailboxFull  ) callbacks.rx_mailbox_full  = callback;
    else if constexpr( event == Event::TxMailboxEmpty ) callbacks.tx_mailbox_empty = callback;
    else details::mailbox_event_no_match();
}

template<Mailbox::Event event>
void Mailbox::unregister_notification() {
    
    // Uncache the target callback in the object
         if constexpr( event == Event::RxMailboxFull  ) callbacks.rx_mailbox_full  = nullptr;
    else if constexpr( event == Event::TxMailboxEmpty ) callbacks.tx_mailbox_empty = nullptr;
    else details::mailbox_event_no_match();
}

/* ==================================================== Protected ctors & dtors =================================================== */

Mailbox::Mailbox(Channel &channel) : 
    channel{ channel } 
{ }

/* =================================================== Private methods (helpers) ================================================== */

template<typename T>
Mailbox::Packet &Mailbox::to_packet(T &obj) {
    return reinterpret_cast<Mailbox::Packet&>(obj);
}


template<typename T>
const Mailbox::Packet &Mailbox::to_packet(const T &obj) {
    return reinterpret_cast<const Mailbox::Packet&>(obj);
}

/* ================================================================================================================================ */

} // End namespace cifx

/* ================================================================================================================================ */

#endif
