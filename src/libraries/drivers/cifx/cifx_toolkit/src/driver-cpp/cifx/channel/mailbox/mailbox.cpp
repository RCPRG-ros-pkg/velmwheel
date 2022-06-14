/* ============================================================================================================================ *//**
 * @file       mailbox.cpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 4th May 2022 12:03:11 pm
 * @modified   Wednesday, 25th May 2022 9:44:59 pm
 * @project    engineering-thesis
 * @brief      Definitions of methods of the class wrapping description and providing related API for the 'Mailbox' concept of 
 *             the CIFX Toolkit Framework
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

/* =========================================================== Includes =========================================================== */

// Private includes
#include "cifx/channel.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace cifx {

/* ================================================ Public methods (Notifications) ================================================ */

namespace details {

    void cifx_mailbox_callback(
        uint32_t notification_event,
        [[maybe_unused]] uint32_t data_len,
        void* data,
        void* context
    ) {

        // Get reference to the @ref Mailbox object
        Mailbox &mailbox = *static_cast<Mailbox*>(context);

        // Dispatch callback
        switch(notification_event) {
                
            // Dispatch handler for the 'RxMailboxFull' event
            case CIFX_NOTIFY_RX_MBX_FULL:

                if(mailbox.callbacks.rx_mailbox_full)
                    mailbox.callbacks.rx_mailbox_full(
                        *static_cast<Mailbox::event_traits<Mailbox::Event::RxMailboxFull>::arg_type*>(data));
            
                break;
                
            // Dispatch handler for the 'TxMailboxEmpty state' event
            case CIFX_NOTIFY_TX_MBX_EMPTY:

                // Check if non-null data is given (as exppected)
                if(data == nullptr)
                    return;

                // Call handler
                if(mailbox.callbacks.tx_mailbox_empty)
                    mailbox.callbacks.tx_mailbox_empty(
                        *static_cast<Mailbox::event_traits<Mailbox::Event::TxMailboxEmpty>::arg_type*>(data));
            
                break;
                
            // Non-supported event
            default: break;
        }

        return;
    }

} // End namespace details

/* ======================================================== Private methods ======================================================= */

void Mailbox::put_packet_impl(const Packet &packet, std::chrono::milliseconds timeout_ms) {

    // Send package to the device
    int32_t ret = xChannelPutPacket(
        channel.handle,
        &const_cast<Packet&>(packet),
        static_cast<uint32_t>(timeout_ms.count())
    );

    // On error throw exception
    if(ret != CIFX_NO_ERROR)
        throw cifx::Error{ ret, "[cifx::Mailbox::put_packet] Failed to send package to the channel" };
}


void Mailbox::get_packet_impl(Packet &packet, std::size_t packet_size, std::chrono::milliseconds timeout_ms) {

    // Send package to the device
    int32_t ret = xChannelGetPacket(
        channel.handle,
        packet_size,
        &packet,
        static_cast<uint32_t>(timeout_ms.count())
    );

    // On error throw exception
    if(ret != CIFX_NO_ERROR)
        throw cifx::Error{ ret, "[cifx::Mailbox::get_packet] Failed to read package from the channel" };
    
}

void Mailbox::exchange_packet_impl(
    const Packet &request,
    Packet &response, 
    std::size_t response_size,
    std::chrono::milliseconds timeout_ms
) {
    // Get start time of exchange
    auto start = std::chrono::steady_clock::now();
    
    int32_t ret;

    // Send package to the device
    ret = xChannelPutPacket(
        channel.handle,
        &const_cast<Packet&>(request),
        static_cast<uint32_t>(timeout_ms.count())
    );

    // On error throw exception
    if(ret != CIFX_NO_ERROR)
        throw cifx::Error{ ret, "[cifx::Mailbox::exchange_packet] Failed to send package to the channel" };

    // Get end time of packet putting
    auto put_end = std::chrono::steady_clock::now();
    // Calculate left timeout
    auto left_timeout_ms = std::chrono::duration_cast<std::chrono::milliseconds>(timeout_ms - (put_end - start));

    // Check if time to get response left
    if(left_timeout_ms.count() <= 0)
        throw cifx::Error{ ret, "[cifx::Mailbox::exchange_packet] Failed to read package from the channel" };

    // Send package to the device
    ret = xChannelGetPacket(
        channel.handle,
        response_size,
        &response,
        static_cast<uint32_t>(left_timeout_ms.count())
    );

    // On error throw exception
    if(ret != CIFX_NO_ERROR)
        throw cifx::Error{ ret, "[cifx::Mailbox::exchange_packet] Failed to read package from the channel" };

    /**
     * @brief Check whether response packet matchses request one. There is always 
     *    differnce of magnitude @c 1 between @a ulCmd fields of the request and response
     *    packet
     * @note This feature is not explicitly expressed by the documentation. It has been
     *    deduced from docuemntation-provided examples of analysed mailbox packages
     */
    if(request.tHeader.ulCmd + 1 != response.tHeader.ulCmd)
        throw cifx::Error{ ret, "[cifx::Mailbox::exchange_packet] Received response does not match sent request" };
}


void Mailbox::exchange_packet_impl(
    const Packet &request,
    Packet &response, 
    std::size_t response_size,
    std::chrono::milliseconds request_timeout_ms,
    std::chrono::milliseconds response_timeout_ms
) {
    
    int32_t ret;
    
    // Send package to the device
    ret = xChannelPutPacket(
        channel.handle,
        &const_cast<Packet&>(request),
        static_cast<uint32_t>(request_timeout_ms.count())
    );

    // On error throw exception
    if(ret != CIFX_NO_ERROR)
        throw cifx::Error{ ret, "[cifx::Mailbox::exchange_packet] Failed to send package to the channel" };

    // Send package to the device
    ret = xChannelGetPacket(
        channel.handle,
        response_size,
        &response,
        static_cast<uint32_t>(response_timeout_ms.count())
    );

    // On error throw exception
    if(ret != CIFX_NO_ERROR)
        throw cifx::Error{ ret, "[cifx::Mailbox::exchange_packet] Failed to read package from the channel" };

    /**
     * @brief Check whether response packet matchses request one. There is always 
     *    differnce of magnitude @c 1 between @a ulCmd fields of the request and response
     *    packet
     * @note This feature is not explicitly expressed by the documentation. It has been
     *    deduced from docuemntation-provided examples of analysed mailbox packages
     */
    if(request.tHeader.ulCmd + 1 != response.tHeader.ulCmd)
        throw cifx::Error{ ret, "[cifx::Mailbox::exchange_packet] Received response does not match sent request" };
}

/* ================================================================================================================================ */

} // End namespace cifx
