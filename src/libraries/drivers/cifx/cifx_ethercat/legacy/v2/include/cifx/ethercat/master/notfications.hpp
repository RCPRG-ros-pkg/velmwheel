/* ============================================================================================================================ *//**
 * @file       notfications.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 20th April 2022 8:44:21 pm
 * @modified   Wednesday, 25th May 2022 9:33:34 pm
 * @project    engineering-thesis
 * @brief      Declarations of helper data structure related to CIFX event notifications
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __CIFX_ETHERCAT_MASTER_NOTIFICATIONS_H__
#define __CIFX_ETHERCAT_MASTER_NOTIFICATIONS_H__

/* =========================================================== Includes =========================================================== */

// System includes
#include <functional>
#include <type_traits>
// CIFX includes
#include "cifXUser.h"

/* ========================================================== Namespaces ========================================================== */

namespace cifx {
namespace ethercat {
namespace notifications {

/* ============================================================= Types ============================================================ */

/**
 * @brief Events that the notification can be registered for
 * @see 'cifx_toolkit/doc/cifX API PR 09 EN.pdf'
 */
enum class Event {
    RxMailboxFull,
    TxMailboxEmpty,
    PD0In,
    PD1In,
    PD0Out,
    PD1Out,
    Sync,
    ComState,
    Num
};

/* ============================================================ Traits ============================================================ */

namespace details {

    template<Event event>
    struct traits : std::false_type { };

    template<>
    struct traits<Event::RxMailboxFull> : std::true_type {
        using arg_type = CIFX_NOTIFY_RX_MBX_FULL_DATA_T;
        using callback_type = std::function<void(const CIFX_NOTIFY_RX_MBX_FULL_DATA_T&)>;
    };

    template<>
    struct traits<Event::TxMailboxEmpty> : std::true_type {
        using arg_type = CIFX_NOTIFY_TX_MBX_EMPTY_DATA_T;
        using callback_type = std::function<void(const CIFX_NOTIFY_TX_MBX_EMPTY_DATA_T&)>;
    };

    template<>
    struct traits<Event::PD0In> : std::true_type {
        using callback_type = std::function<void(void)>;
    };

    template<>
    struct traits<Event::PD1In> : std::true_type {
        using callback_type = std::function<void(void)>;
    };

    template<>
    struct traits<Event::PD0Out> : std::true_type {
        using callback_type = std::function<void(void)>;
    };

    template<>
    struct traits<Event::PD1Out> : std::true_type {
        using callback_type = std::function<void(void)>;
    };

    template<>
    struct traits<Event::Sync> : std::true_type {
        using callback_type = std::function<void(void)>;
    };

    template<>
    struct traits<Event::ComState> : std::true_type {
        using arg_type = CIFX_NOTIFY_COM_STATE_T;
        using callback_type = std::function<void(const CIFX_NOTIFY_COM_STATE_T&)>;
    };

}


/**
 * @brief Helper structure providing traits of the given notifiable @tparam event 
 * 
 * @tparam event 
 *    vent to get traits for
 */
template<Event event>
struct traits : details::traits<event> { 
    static_assert(details::traits<event>::value, "[cifx::ethercat::notifications] Traits instantiated for invalid Event"); 
};


/**
 * @brief Helper alias for the type of the callback functor of the given notifiable @tparam event 
 * 
 * @tparam event 
 *    vent to get callback type for
 */
template<Event event>
using callback_type = typename traits<event>::callback_type;


/**
 * @brief Helper alias for the type of the argument of the callback functor of the given notifiable
 *   @tparam event 
 * 
 * @tparam event 
 *    vent to get callback type for
 */
template<Event event>
using arg_type = typename traits<event>::arg_type;

/* ============================================================= Types ============================================================ */

/**
 * @brief Structure holding set of callbacks registered for CIFX events
 */
struct Callbacks {

    // Callback handler for the 'RX mailbox full' event
    notifications::callback_type<notifications::Event::RxMailboxFull> rx_mbx_full;
    // Callback handler for the 'TX mailbox full' event
    notifications::callback_type<notifications::Event::TxMailboxEmpty> tx_mbx_empty;
    // Callback handler for the 'Input area 0 has been processed' event
    notifications::callback_type<notifications::Event::PD0In> pd0_in;
    // Callback handler for the 'Input area 1 has been processed' event
    notifications::callback_type<notifications::Event::PD1In> pd1_in;
    // Callback handler for the 'Output area 0 has been processed' event
    notifications::callback_type<notifications::Event::PD0Out> pd0_out;
    // Callback handler for the 'Output area 1 has been processed' event
    notifications::callback_type<notifications::Event::PD1Out> pd1_out;
    // Callback handler for the 'Sync' event
    notifications::callback_type<notifications::Event::Sync> sync;
    // Callback handler for the 'Communication state' event
    notifications::callback_type<notifications::Event::ComState> com_state;

};

/* ========================================================= Free functions ======================================================= */

/**
 * @brief Converts @tparam event to corresponding CIFX flag
 * 
 * @tparam event 
 *    event to be converted
 * @returns 
 *    CIFX Toolkit representation of the @tparam event
 */
template<notifications::Event event>
constexpr inline uint32_t event_to_cifx();

/**
 * 
 * @tparam event 
 *    event to be converted
 * @returns 
 *    refere to the @p callback's member corresponding to the @tparam event
 */
template<notifications::Event event>
inline notifications::callback_type<event> &event_to_callback(Callbacks &callbask);

/**
 * @brief C-like callback function used to register CIFX-specific notification callbacks with the @ref Master 
 *    driver. 
 * @details Function implements signature of the @ref PFN_NOTIFY_CALLBACK type that is required by 
 *    @ref xChannelRegisterNotification(...) toolchain function to register callback handler for the given
 *    notification event. The function is wrapper calling @ref std::function handler from the @ref Master
 *    object for the given @p notification_event .
 * 
 * @tparam Lock
 *    lock type of the master object
 * 
 * @param notification_event 
 *    notification event to be handled
 * @param data_len 
 *    length of the data associated with the notification
 * @param data 
 *    data associated with the notification
 * @param context 
 *    pointer to the @ref Master object that the handler has been registered for
 */
template<typename Lock>
void cifx_callback(
    uint32_t notification_event,
    uint32_t data_len,
    void* data,
    void* context
);

/* ================================================================================================================================ */

} // End namespace notifications
} // End namespace ethercat
} // End namespace cifx

/* ==================================================== Implementation includes =================================================== */

#include "cifx/ethercat/master/notifications/notifications.hpp"

/* ================================================================================================================================ */

#endif
