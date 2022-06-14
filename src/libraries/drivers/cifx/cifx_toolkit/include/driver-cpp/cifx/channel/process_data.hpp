/* ============================================================================================================================ *//**
 * @file       process_data.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 4th May 2022 12:10:47 pm
 * @modified   Wednesday, 25th May 2022 9:36:16 pm
 * @project    engineering-thesis
 * @brief      Definition of the class wrapping description and providing related API for the 'Process Data' concept of the CIFX 
 *             Toolkit Framework
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __CIFX_CHANNEL_PROCESS_DATA_H__
#define __CIFX_CHANNEL_PROCESS_DATA_H__

/* =========================================================== Includes =========================================================== */

// System includes
#include <mutex>
// External includes
#include "range/v3/span.hpp"
// CIFX includes
#include "cifxDriver.h"
#include "cifXHWFunctions.h"

/* ========================================================== Namespaces ========================================================== */

namespace cifx {

/* ========================================================= Declarations ========================================================= */

namespace details {
    
    void cifx_process_data_callback(
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
 * @brief Class wrapping description and providing related API for the 'Process Data' concept of the CIFX 
 *    Toolkit Framework
 */
class ProcessData {

    /// Make Channel class a friend to let it access constructor
    friend class Channel;

public: /* ------------------------------------------------------ Public types ---------------------------------------------------- */

    /**
     * @brief Enumeration of the available process data areas
     */
    enum class Area : uint32_t {

        /// Regular Process Data Area (PD0)
        Regular = 0,

        /// Prioritized Process Data Area (PD1) [not implemented in CIFX toolkit]
        Priority = 1

    };

    /**
     * @brief Events related to the Channel's ProcessData that the notification can be 
     *    registered for
     * @see 'cifx_toolkit/doc/cifX API PR 09 EN.pdf'
     */
    enum class Event {
        Input,
        Output
    };

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
    void register_notification(const std::function<void(void)> &callback);

    /**
     * @brief Unregisters callback fot eh given @p event
     * 
     * @tparam event 
     *    event to unregister callback for
     */
    template<Event event>
    void unregister_notification();

public: /* -------------------------------------------- Public methods (Synchronous I/O) ------------------------------------------ */

    /**
     * @brief Reads process data from the associated data area the channel at the given @p offset
     * 
     * @param[in] offset 
     *    offset of the data to be read from the beggining of the @p area
     * @param[out] buffer 
     *    buffer to read data into
     * @param[in] timeout_ms 
     *    timeout of the 'read' action
     * 
     * @throws cifx::Error 
     *    on failure
     * 
     * @synchronised
     */
    inline void read(
        std::size_t offset,
        ranges::span<uint8_t> buffer,
        std::chrono::milliseconds timeout_ms
    );

    /**
     * @brief Writes process data to the associated data area the channel at the given @p offset
     * 
     * @param[in] offset 
     *    offset of the data to be written from the beggining of the @p area
     * @param[out] buffer 
     *    buffer to write data from
     * @param[in] timeout_ms 
     *    timeout of the 'write' action
     * 
     * @throws cifx::Error 
     *    on failure
     * 
     * @synchronised
     */
    inline void write(
        std::size_t offset,
        ranges::span<const uint8_t> buffer,
        std::chrono::milliseconds timeout_ms
    );

protected: /* ------------------------------------------------- Protected constructors -------------------------------------------- */

    /**
     * @brief Constructs a new object wrapping description and providing related API 
     *     for the 'Device's Channel' concept of the CIFX Toolkit Framework
     * 
     * @param channel 
     *    handle to the associated channel
     * @param area 
     *    index of the data area to be written (currently, CIFX toolkit supports only 
     *    @c Area::Regular area )
     * 
     * @throws Error
     *    when the procedure failed to open the channel handle in the toolkit
     */
    inline ProcessData(Channel &channel, Area area = Area::Regular);

    /**
     * @brief Destroys the Channel object closing related communication channel
     */
    inline ~ProcessData() = default;

private: /* ---------------------------------------------------- Private friends -------------------------------------------------- */
    
    /// Make CIFX-specific interface notification callback function a friend to let it access notifications
    friend void details::cifx_process_data_callback(
        uint32_t notification_event,
        uint32_t data_len,
        void* data,
        void* context
    );

private: /* ----------------------------------------------- Private member variables ---------------------------------------------- */

    /// Handle to the associated channel
    Channel &channel;
    /// ID of the associated data area
    Area area;

    /// Notification callbacks
    struct {

        /// Callback registered for @c Event::Input event
        std::function<void(void)> input;
        /// Callback registered for @c Event::Output event
        std::function<void(void)> output;

    } callbacks;
    
};

/* ================================================================================================================================ */

} // End namespace cifx

/* ================================================================================================================================ */

#endif
