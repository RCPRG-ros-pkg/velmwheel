/* ============================================================================================================================ *//**
 * @file       device.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 20th April 2022 8:06:44 pm
 * @modified   Thursday, 28th April 2022 5:46:27 pm
 * @project    engineering-thesis
 * @brief      Definitions of public device configuration methods of the the EtherCAT Master class template
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __CIFX_ETHERCAT_MASTER_IMPL_DEVICE_H__
#define __CIFX_ETHERCAT_MASTER_IMPL_DEVICE_H__

/* =========================================================== Includes =========================================================== */

// System includes
#include <cstdint>
#include <mutex>
// Boost includes
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
// CIFX includes
#include "Hil_ApplicationCmd.h"
#include "EcmIF_Public.h"
// Private includes
#include "cifx/ethercat/master.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace cifx {
namespace ethercat {

/* =========================================================== Constants ========================================================== */

/**
 * @brief Helper structure defining values of the key fields of the @ref HIL_SET_HANDSHAKE_CONFIG_REQ_DATA_T 
 *    structure for configuring given synchronisation mode of the CIFX channel
 * 
 * @see 'doc/EtherCAT Master V4 Protocol API 06 EN.pdf', p. 24/254
 */
struct SyncModeConfig {
    uint8_t bPDInHskMode;
    uint8_t bPDInSource;
    uint8_t bPDOutHskMode;
    uint8_t bPDOutSource;
};

// Configuration of registers for the Free-Run synchronisation mode
constexpr SyncModeConfig FreeRunSyncModeConfig { 0, 0, 0, 0 };
// Configuration of registers for the I/O Sync Mode 1 synchronisation mode
constexpr SyncModeConfig IO1SyncModeConfig { 5, 0, 4, 0 };
// Configuration of registers for the I/O Sync Mode 2 synchronisation mode
constexpr SyncModeConfig IO2SyncModeConfig { 6, 0, 4, 0 };

/* ============================================= Public methods (device configuration) ============================================ */

template<typename Lock>
const std::string &Master<Lock>::get_eni_path() const noexcept {
    return eni_path;
}


template<typename Lock>
const std::chrono::nanoseconds &Master<Lock>::get_bus_cycle_time() const noexcept {
    return bus_cycle;
}


template<typename Lock>
void Master<Lock>::set_master_ready(bool ready) {
    std::lock_guard<Lock> glock(lock);
    set_master_ready_impl(ready);
}


template<typename Lock>
bool Master<Lock>::is_master_ready() {

    std::lock_guard<Lock> glock(lock);

    uint32_t state;

    // Get master state known by the device
    auto status = xChannelHostState(
        cifx_channel,
        CIFX_HOST_STATE_READ,
        &state,
        static_cast<uint32_t>(timeouts[to_underlying(TimeoutAction::DeviceCommunication)].count())
    );

    // On error throw exception
    if(status != CIFX_NO_ERROR)
        throw cifx::Error{ status, "[cifx::ethercat::Master::is_master_ready] Failed to communicate with the device" };

    return (state == CIFX_HOST_STATE_READ);
}


template<typename Lock>
void Master<Lock>::set_bus_on(bool ready) {
    std::lock_guard<Lock> glock(lock);
    set_bus_on_impl(ready);
}


template<typename Lock>
bool Master<Lock>::is_bus_on() {

    std::lock_guard<Lock> glock(lock);

    uint32_t state;

    // Get bus state
    auto status = xChannelBusState(
        cifx_channel,
        CIFX_BUS_STATE_GETSTATE,
        &state,
        static_cast<uint32_t>(timeouts[to_underlying(TimeoutAction::DeviceCommunication)].count())
    );

    // On error throw exception
    if(status != CIFX_NO_ERROR)
        throw cifx::Error{ status, "[cifx::ethercat::Master::is_bus_on] Failed to communicate with the device" };

    return (state == CIFX_BUS_STATE_ON);
}


template<typename Lock>
void Master<Lock>::configure_sync_mode(SyncMode mode) {

    std::lock_guard<Lock> glock(lock);

    /* ------------------------- Send request to the device -------------------------- */

    // Select configuration to be set
    SyncModeConfig target_config;
    switch(mode) {
        case SyncMode::FreeRun: target_config = FreeRunSyncModeConfig; break;
        case SyncMode::IO1:     target_config = IO1SyncModeConfig;     break;
        case SyncMode::IO2:     target_config = IO2SyncModeConfig;     break;
        default:
            throw cifx::Error{ 
                CIFX_INVALID_PARAMETER,
                "[cifx::ethercat::Master::configure_sync_mode] Invalid SyncMode given" 
            };
    }
    
    // Prepare request (zero-initialize)
    HIL_SET_HANDSHAKE_CONFIG_REQ_T req { };
    // Prepare request's header
    req.tHead.ulDest = HIL_PACKET_DEST_DEFAULT_CHANNEL;
    req.tHead.ulLen  = 20;
    req.tHead.ulId   = 1;
    req.tHead.ulSta  = 0;
    req.tHead.ulCmd  = HIL_SET_HANDSHAKE_CONFIG_REQ;
    req.tHead.ulExt  = 0;
    req.tHead.ulRout = 0;
    // Prepare request's body
    req.tData.bPDInHskMode   = target_config.bPDInHskMode;
    req.tData.bPDInSource    = target_config.bPDInSource;
    req.tData.bPDOutHskMode  = target_config.bPDOutHskMode;
    req.tData.bPDOutSource   = target_config.bPDOutSource;
    req.tData.usPDOutErrorTh = 0;
    req.tData.usPDInErrorTh  = 0;
    req.tData.bSyncHskMode   = 0;
    req.tData.bSyncSource    = 0;
    req.tData.usSyncErrorTh  = 0;

    uint32_t status;

    // Send request package to the device
    status = xChannelPutPacket(
        cifx_channel,
        reinterpret_cast<CIFX_PACKET*>(&req),
        static_cast<uint32_t>(timeouts[to_underlying(TimeoutAction::PutPacket)].count())
    );

    // On error throw exception
    if(status != CIFX_NO_ERROR)
        throw cifx::Error{ status, "[cifx::ethercat::Master::configure_sync_mode] Failed to send request package" };

    /* ---------------------- Receive response from the device ----------------------- */

    HIL_SET_HANDSHAKE_CONFIG_CNF_T res;

    // Receive response package from the device
    status = xChannelGetPacket(
        cifx_channel,
        sizeof(res),
        reinterpret_cast<CIFX_PACKET*>(&res),
        static_cast<uint32_t>(timeouts[to_underlying(TimeoutAction::GetPacket)].count())
    );

    // On error throw exception
    if(status != CIFX_NO_ERROR)
        throw cifx::Error{ status, "[cifx::ethercat::Master::configure_sync_mode] Failed to received response package" };

    /* ------------------------------- Parse response -------------------------------- */        

    // On errorronous response throw exception
    if(res.tHead.ulSta != CIFX_NO_ERROR)
        throw cifx::Error{ status, "[cifx::ethercat::Master::configure_sync_mode] Failed to configure synchronisation mode" };

}


template<typename Lock>
TimingInfo Master<Lock>::get_timing_info() {

    std::lock_guard<Lock> glock(lock);

    /* ------------------------- Send request to the device -------------------------- */
    
    // Prepare request (zero-initialize)
    ECM_IF_GET_TIMING_INFO_REQ_T req { };
    // Prepare request's header
    req.tHead.ulDest = HIL_PACKET_DEST_DEFAULT_CHANNEL;
    req.tHead.ulLen  = 0;
    req.tHead.ulId   = 1;
    req.tHead.ulSta  = 0;
    req.tHead.ulCmd  = ECM_IF_CMD_GET_TIMING_INFO_REQ;
    req.tHead.ulExt  = 0;
    req.tHead.ulRout = 0;

    uint32_t status;
    
    // Send request package to the device
    status = xChannelPutPacket(
        cifx_channel,
        reinterpret_cast<CIFX_PACKET*>(&req),
        static_cast<uint32_t>(timeouts[to_underlying(TimeoutAction::PutPacket)].count())
    );

    // On error throw exception
    if(status != CIFX_NO_ERROR)
        throw cifx::Error{ status, "[cifx::ethercat::Master::get_timing_info] Failed to send request package" };

    /* ---------------------- Receive response from the device ----------------------- */

    ECM_IF_GET_TIMING_INFO_CNF_T res;

    // Receive response package from the device
    status = xChannelGetPacket(
        cifx_channel,
        sizeof(res),
        reinterpret_cast<CIFX_PACKET*>(&res),
        static_cast<uint32_t>(timeouts[to_underlying(TimeoutAction::GetPacket)].count())
    );

    // On error throw exception
    if(status != CIFX_NO_ERROR)
        throw cifx::Error{ status, "[cifx::ethercat::Master::get_timing_info] Failed to received response package" };

    /* ------------------------------- Parse response -------------------------------- */        

    // On errorronous response throw exception
    if(res.tHead.ulSta != CIFX_NO_ERROR)
        throw cifx::Error{ status, "[cifx::ethercat::Master::get_timing_info] Failed to configure synchronisation mode" };

    // Get timing info from response apckage
    ECM_IF_GET_TIMING_INFO_CNF_DATA_T timing_data = res.tData;

    // Return timing info
    return TimingInfo {
        .bus_cycle              { std::chrono::nanoseconds(timing_data.ulBusCycleTimeNs)       },
        .frame_transmition_time { std::chrono::nanoseconds(timing_data.ulFrameTransmitTimeNs)  },
        .expected_bus_delay     { std::chrono::nanoseconds(timing_data.ulExpectedBusDelayNs)   },
        .expected_rx_end_time   { std::chrono::nanoseconds(timing_data.ulExpectedRxEndTimeNs)  },
        .expected_tx_end_time   { std::chrono::nanoseconds(timing_data.ulExpectedTxDataTimeNs) }
    };
}

/* ======================================================== Private methods ======================================================= */


template<typename Lock>
void Master<Lock>::set_master_ready_impl(bool ready) {

    uint32_t state;

    // Get master state known by the device
    auto status = xChannelHostState(
        cifx_channel,
        ready ? CIFX_HOST_STATE_READY : CIFX_HOST_STATE_NOT_READY,
        &state,
        static_cast<uint32_t>(timeouts[to_underlying(TimeoutAction::DeviceCommunication)].count())
    );

    // On error throw exception
    if(status != CIFX_NO_ERROR)
        throw cifx::Error{ status, "[cifx::ethercat::Master::set_master_ready_impl] Failed to communicate with the device" };
}


template<typename Lock>
void Master<Lock>::set_bus_on_impl(bool ready) {

    uint32_t state;

    // Get bus state
    auto status = xChannelBusState(
        cifx_channel,
        ready ? CIFX_BUS_STATE_ON : CIFX_BUS_STATE_OFF,
        &state,
        static_cast<uint32_t>(timeouts[to_underlying(TimeoutAction::DeviceCommunication)].count())
    );

    // On error throw exception
    if(status != CIFX_NO_ERROR)
        throw cifx::Error{ status, "[cifx::ethercat::Master::set_bus_on_impl] Failed to communicate with the device" };
}

/* ================================================================================================================================ */

} // End namespace ethercat
} // End namespace cifx

#endif

