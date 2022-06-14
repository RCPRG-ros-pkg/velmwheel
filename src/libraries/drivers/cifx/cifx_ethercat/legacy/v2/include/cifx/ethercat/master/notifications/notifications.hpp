/* ============================================================================================================================ *//**
 * @file       notifications.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 20th April 2022 8:06:44 pm
 * @modified   Wednesday, 27th April 2022 3:48:50 pm
 * @project    engineering-thesis
 * @brief      Definitions of helper data structure related to CIFX event notifications
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __CIFX_ETHERCAT_MASTER_NOTIFICATIONS_NOTIFICATIONS_H__
#define __CIFX_ETHERCAT_MASTER_NOTIFICATIONS_NOTIFICATIONS_H__

/* =========================================================== Includes =========================================================== */

// System includes
#include <cstdint>
// Private includes
#include "cifx/ethercat/master.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace cifx {
namespace ethercat {
namespace notifications {

/* ======================================================= Helper functions ======================================================= */

template<bool flag = false>
void event_no_match() {
    static_assert(flag, "[cifx::ethercat::notifications] Invalid event"); 
}

/* ======================================================== Free functions ======================================================== */

template<notifications::Event event>
constexpr uint32_t event_to_cifx() {
         if constexpr( event == Event::RxMailboxFull  ) return CIFX_NOTIFY_RX_MBX_FULL;
    else if constexpr( event == Event::TxMailboxEmpty ) return CIFX_NOTIFY_TX_MBX_EMPTY;
    else if constexpr( event == Event::PD0In          ) return CIFX_NOTIFY_PD0_IN;
    else if constexpr( event == Event::PD1In          ) return CIFX_NOTIFY_PD1_IN;
    else if constexpr( event == Event::PD0Out         ) return CIFX_NOTIFY_PD0_OUT;
    else if constexpr( event == Event::PD1Out         ) return CIFX_NOTIFY_PD1_OUT;
    else if constexpr( event == Event::Sync           ) return CIFX_NOTIFY_SYNC;
    else if constexpr( event == Event::ComState       ) return CIFX_NOTIFY_COM_STATE;
    else event_no_match();
}


template<notifications::Event event>
notifications::callback_type<event> &event_to_callback(Callbacks &callback) {
         if constexpr( event == Event::RxMailboxFull  ) return callback.rx_mbx_full;
    else if constexpr( event == Event::TxMailboxEmpty ) return callback.tx_mbx_empty;
    else if constexpr( event == Event::PD0In          ) return callback.pd0_in;
    else if constexpr( event == Event::PD1In          ) return callback.pd1_in;
    else if constexpr( event == Event::PD0Out         ) return callback.pd0_out;
    else if constexpr( event == Event::PD1Out         ) return callback.pd1_out;
    else if constexpr( event == Event::Sync           ) return callback.sync;
    else if constexpr( event == Event::ComState       ) return callback.com_state;
    else event_no_match();
}


template<typename Lock>
void cifx_callback(
    uint32_t notification_event,
    [[maybe_unused]] uint32_t data_len,
    void* data,
    void* context
) {

    // Get reference to the @ref Master object
    Master<Lock> &master = *static_cast<Master<Lock>*>(context);

    // Dispatch callback
    switch(notification_event) {
        
        // Dispatch handler for the 'RX mailbox full' event
        case CIFX_NOTIFY_RX_MBX_FULL:

            // Check if non-null data is given (as exppected)
            if(data == nullptr)
                return;

            // Call handler
            if(master.callbacks.rx_mbx_full)
                master.callbacks.rx_mbx_full(
                    *static_cast<arg_type<Event::RxMailboxFull>*>(data));
        
            break;
            
        // Dispatch handler for the 'TX mailbox full' event
        case CIFX_NOTIFY_TX_MBX_EMPTY:

            // Check if non-null data is given (as exppected)
            if(data == nullptr)
                return;

            // Call handler
            if(master.callbacks.tx_mbx_empty)
                master.callbacks.tx_mbx_empty(
                    *static_cast<arg_type<Event::TxMailboxEmpty>*>(data));
        
            break;
            
        // Dispatch handler for the 'Input area 0 has been processed' event
        case CIFX_NOTIFY_PD0_IN:

            if(master.callbacks.pd0_in)
                master.callbacks.pd0_in();
        
            break;
            
        // Dispatch handler for the 'Input area 1 has been processed' event
        case CIFX_NOTIFY_PD1_IN:

            if(master.callbacks.pd1_in)
                master.callbacks.pd1_in();
        
            break;
            
        // Dispatch handler for the 'Output area 0 has been processed' event
        case CIFX_NOTIFY_PD0_OUT:

            if(master.callbacks.pd0_out)
                master.callbacks.pd0_out();
        
            break;
            
        // Dispatch handler for the 'Output area 1 has been processed' event
        case CIFX_NOTIFY_PD1_OUT:

            if(master.callbacks.pd1_out)
                master.callbacks.pd1_out();
        
            break;
            
        // Dispatch handler for the 'Sync' event
        case CIFX_NOTIFY_SYNC:

            if(master.callbacks.sync)
                master.callbacks.sync();
        
            break;
            
        // Dispatch handler for the 'Communication state' event
        case CIFX_NOTIFY_COM_STATE:

            // Check if non-null data is given (as exppected)
            if(data == nullptr)
                return;

            // Call handler
            if(master.callbacks.com_state)
                master.callbacks.com_state(
                    *static_cast<arg_type<Event::ComState>*>(data));
        
            break;
            
        // Non-supported event
        default: break;
    }

    return;
}

/* ================================================================================================================================ */

} // End namespace notifications
} // End namespace ethercat
} // End namespace cifx

#endif
