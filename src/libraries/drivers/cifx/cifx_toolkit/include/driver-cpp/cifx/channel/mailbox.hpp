/* ============================================================================================================================ *//**
 * @file       mailbox.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 4th May 2022 12:10:47 pm
 * @modified   Thursday, 30th June 2022 1:47:29 pm
 * @project    engineering-thesis
 * @brief      Definition of the class wrapping description and providing related API for the 'Mailbox' concept of the CIFX 
 *             Toolkit Framework
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __CIFX_CHANNEL_MAILBOX_H__
#define __CIFX_CHANNEL_MAILBOX_H__

/* =========================================================== Includes =========================================================== */

// System includes
#include <mutex>
#include <type_traits>
// External includes
#include "range/v3/span.hpp"
// CIFX includes
#include "cifxDriver.h"
#include "cifXHWFunctions.h"

/* ========================================================== Namespaces ========================================================== */

namespace cifx {

/* ========================================================= Declarations ========================================================= */

namespace details {
    
    void cifx_mailbox_callback(
        uint32_t notification_event,
        uint32_t data_len,
        void* data,
        void* context
    );

} // End namespace details

// Forward declare channel class
class Channel;
    
/* ============================================================= Class ============================================================ */

/**
 * @brief Class wrapping description and providing related API for the 'Mailbox' concept of the CIFX 
 *    Toolkit Framework
 */
class Mailbox {

    /// Make Channel class a friend to let it access constructor
    friend class Channel;

public: /* ------------------------------------------------------ Public types ---------------------------------------------------- */

    /**
     * @brief Events related to the Channel's Mailbox that the notification can be 
     *    registered for
     * @see 'cifx_toolkit/doc/cifX API PR 09 EN.pdf'
     */
    enum class Event {
        RxMailboxFull,
        TxMailboxEmpty
    };

    /// Type of the callback associated with the @c Event::RxMailboxFull callback
    using RxMailboxFullCallback = std::function<void(const CIFX_NOTIFY_RX_MBX_FULL_DATA_T&)>;
    /// Type of the callback associated with the @c Event::TxMailboxEmpty callback
    using TxMailboxEmptyCallback = std::function<void(const CIFX_NOTIFY_TX_MBX_EMPTY_DATA_T&)>;

    /// Traits of the notification event ( used to define argument type for @ref register_notification(...) )
    template<Event event> struct event_traits : public std::false_type { };

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

public: /* ------------------------------------------- Public methods (Asynchronous I/O) ------------------------------------------ */

    /**
     * @brief Puts @p packet into the mailbox of the channel
     * 
     * @tparam PacketT 
     *    type of the @p packet 
     * @param packet 
     *    packet to be put into the mailbox
     * @param timeout_ms 
     *    timeout of the 'put' action
     * 
     * @throws cifx::Error 
     *    on failure
     * 
     * @note @p packet reference is converted to the CIFX-specific packet reference
     *    undethehood
     */
    template<typename PacketT>
    inline void put_packet(const PacketT &packet, std::chrono::milliseconds timeout_ms);

    /**
     * @brief Reads packet from the mailbox of the channel
     * 
     * @tparam PacketT 
     *    type of the @p packet 
     * @param[out] packet 
     *    packet structure to read incominng package into
     * @param[in] timeout_ms 
     *    timeout of the 'get' action
     * 
     * @throws cifx::Error 
     *    on failure
     * 
     * @note @p packet reference is converted to the CIFX-specific packet reference
     *    undethehood
     */
    template<typename PacketT>
    inline void get_packet(PacketT &packet, std::chrono::milliseconds timeout_ms);

    /**
     * @brief Writes request packet to the mailbox and reads the response in an atomic manner
     * 
     * @tparam RequestT 
     *    type of the @p request 
     * @tparam ResponseT 
     *    type of the @p response 
     * @param[in] request 
     *    packet to be put into the mailbox
     * @param[out] response 
     *    packet structure to read incominng package into
     * @param[in] imeout_ms 
     *    timeout of the whole action 
     * 
     * @throws cifx::Error 
     *    on failure
     * 
     * @note @p packet reference is converted to the CIFX-specific packet reference undethehood
     * @warning This function is not reentrant. It follows two-step process to exchange packet
     *    (putting request into the output mailbox and waiting for receiving packet in the input
     *    mailbox). If another thread triesto receive package after the function put it's request
     *    but before it locks on reception, the other thread will receive package that does not
     *    correspond to it's request. In sucha a case function will throw exception with message
     *    stating that the response package does not correspond to request type.
     *    This issue wasn't taken as a problem during initial design phase, as the whole low-level
     *    communication mechanism provided by the library was intended to be managed by a single 
     *    thread. However, if the future expririence shows that the reentrant access is required, 
     *    two-way approach can be taken to enable it:
     *  
     *      * (easier) introduce mailbox-synchronisation lock that would ensure that
     *        packages' exchange is atomic
     *      * (harder) introduce independent mailbox-handling mechanism based on the
     *        'notifications' functionality; in such a case, a dedicated thread created 
     *        in the constructor would manage actual transmission and reception of messages
     *        on behalf of requesting threads taking advantage of IPC (Intra-Process Communication)
     */
    template<typename RequestT, typename ResponseT>
    inline void exchange_packet(
        const RequestT &request,
        ResponseT &response,
        std::chrono::milliseconds timeout_ms
    );

    /**
     * @brief Writes request packet to the mailbox and reads the response in an atomic manner
     * 
     * @tparam RequestT 
     *    type of the @p request 
     * @tparam ResponseT 
     *    type of the @p response 
     * @param[in] request 
     *    packet to be put into the mailbox
     * @param[out] response 
     *    packet structure to read incominng package into
     * @param[in] request_timeout_ms 
     *    timeout of the 'put' action
     * @param[in] response_timeout_ms 
     *    timeout of the 'get' action
     * 
     * @throws cifx::Error 
     *    on failure
     * 
     * @note @p packet reference is converted to the CIFX-specific packet reference undethehood
     * @warning See warning for the @ref exchange_packet() method
     */
    template<typename RequestT, typename ResponseT>
    inline void exchange_packet(
        const RequestT &request,
        ResponseT &response,
        std::chrono::milliseconds request_timeout_ms,
        std::chrono::milliseconds response_timeout_ms
    );

    /**
     * @brief Writes request packet to the mailbox and reads the response in an atomic manner. If timeout
     *    is not reached decreases @p timeout_ms value by time spent on packet exchange
     * 
     * @tparam RequestT 
     *    type of the @p request 
     * @tparam ResponseT 
     *    type of the @p response 
     * @param[in] request 
     *    packet to be put into the mailbox
     * @param[out] response 
     *    packet structure to read incominng package into
     * @param[inout] imeout_ms 
     *    timeout of the whole action; on success decreased by actual time spent on exchange action
     * 
     * @throws cifx::Error 
     *    on failure
     * 
     * @note @p packet reference is converted to the CIFX-specific packet reference undethehood
     * @warning See warning for the @ref exchange_packet() method
     */
    template<typename RequestT, typename ResponseT>
    void exchange_packet_with_timeout_update(
        const RequestT &request,
        ResponseT &response,
        std::chrono::milliseconds &timeout_ms
    );

    /**
     * @brief Writes request packet to the mailbox and reads the response in an atomic manner. If timeout
     *    is not reached decreases @p timeout_ms value by time spent on packet exchange. Updates header of the 
     *    @p request with the @a ulDestId read form the response
     * 
     * @note This method is aimed to perform multi-part packets exchange
     * 
     * @tparam RequestT 
     *    type of the @p request 
     * @tparam ResponseT 
     *    type of the @p response 
     * @param[inout] request 
     *    packet to be put into the mailbox
     * @param[out] response 
     *    packet structure to read incominng package into
     * @param[inout] imeout_ms 
     *    timeout of the whole action; on success decreased by actual time spent on exchange action
     * 
     * @throws cifx::Error 
     *    on failure
     * 
     * @note @p packet reference is converted to the CIFX-specific packet reference undethehood
     * @warning See warning for the @ref exchange_packet() method
     */
    template<typename RequestT, typename ResponseT>
    inline void exchange_partial_packet(
        RequestT &request,
        ResponseT &response,
        std::chrono::milliseconds &timeout_ms
    );


protected: /* ------------------------------------------------- Protected constructors -------------------------------------------- */

    /**
     * @brief Constructs a new object wrapping description and providing related API for 
     *    the 'Mailbox' concept of the CIFX Toolkit Framework
     * 
     * @param channel 
     *    handle to the CIFX channel that the mailbox refers to
     * 
     * @throws Error
     *    when the procedure failed to open the channel handle in the toolkit
     */
    inline Mailbox(Channel &channel);

    /**
     * @brief Destroys the Mailbox object
     */
    inline ~Mailbox() = default;

private: /* ----------------------------------------------------- Private types --------------------------------------------------- */
    
    // Local typename for the CIFX Packet type
    using Packet = CIFX_PACKET;
    
private: /* ----------------------------------------------- Private methods (helpers) --------------------------------------------- */

    /**
     * @brief Converts reference to the @p obj object to the reference to CIFX Packet.
     *   This function is aimed to avoid explicit static casts between references to
     *   the packet instance and packet reference
     * 
     * @tparam T 
     *    type of the obejct ot be converted
     * @param obj 
     *    reference to the obejct to be converted
     * @returns 
     *    reference to CIFX Packet
     */
    template<typename T>
    static inline Packet &to_packet(T &obj);

    /**
     * @overload Packet &to_packet(T &obj)
     * @brief Converts reference to the @p obj object to the reference to CIFX Packet.
     *   This function is aimed to avoid explicit static casts between references to
     *   the packet instance and packet reference
     * 
     * @tparam T 
     *    type of the obejct ot be converted
     * @param obj 
     *    reference to the obejct to be converted
     * @returns 
     *    reference to CIFX Packet
     */
    template<typename T>
    static inline const Packet &to_packet(const T &obj);
    
private: /* ------------------------------------------- Private methods (implementations) ----------------------------------------- */

    /// @brief Internal implementation of @ref put_packet
    void put_packet_impl(const Packet &packet, std::chrono::milliseconds timeout_ms);

    /// @brief Internal implementation of @ref get_packet
    void get_packet_impl(Packet &packet, std::size_t packet_size, std::chrono::milliseconds timeout_ms);

    /// @brief Internal implementation of @ref exchange_packet
    void exchange_packet_impl(
        const Packet &request,
        Packet &response,
        std::size_t response_size,
        std::chrono::milliseconds timeout_ms
    );

    /// @brief Internal implementation of @ref exchange_packet
    void exchange_packet_impl(
        const Packet &request,
        Packet &response,
        std::size_t response_size,
        std::chrono::milliseconds request_timeout_ms,
        std::chrono::milliseconds response_timeout_ms
    );

private: /* ---------------------------------------------------- Private friends -------------------------------------------------- */
    
    /// Make CIFX-specific interface notification callback function a friend to let it access notifications
    friend void details::cifx_mailbox_callback(
        uint32_t notification_event,
        uint32_t data_len,
        void* data,
        void* context
    );

private: /* ----------------------------------------------- Private member variables ---------------------------------------------- */

    /// Handle to the associated channel
    Channel &channel;

    /// Notification callbacks
    struct {

        /// Callback registered for @c Event::RxMailboxFull event
        RxMailboxFullCallback rx_mailbox_full;
        /// Callback registered for @c Event::TxMailboxEmpty event
        TxMailboxEmptyCallback tx_mailbox_empty;

    } callbacks;
    
};

/* ================================================================================================================================ */

} // End namespace cifx

/* ================================================================================================================================ */

#endif
