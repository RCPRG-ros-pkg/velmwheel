/* ============================================================================================================================ *//**
 * @file       master.cpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 26th May 2022 10:25:27 pm
 * @modified   Friday, 27th May 2022 1:15:32 am
 * @project    engineering-thesis
 * @brief      Definition of methods of the Slave class representing Master device on the EtherCAT bus
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

/* =========================================================== Includes =========================================================== */

// CIFX includes
#include "Hil_ApplicationCmd.h"
#include "EcmIF_Public.h"
#include "Hil_Results.h"
// Private includes
#include "cifx/ethercat/master.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace cifx::ethercat {

/* ===================================================== Public common methods ==================================================== */

namespace details {

    static inline Master::ExtendedState cifx_to_state(uint8_t state, std::string_view context) {

        // Convert the state
        switch(state) {
            case ECM_IF_STATE_INIT:             return Master::ExtendedState::Init;
            case ECM_IF_STATE_PREOP:            return Master::ExtendedState::Preop;
            case ECM_IF_STATE_SAFEOP:           return Master::ExtendedState::Safeop;
            case ECM_IF_STATE_OP:               return Master::ExtendedState::Op;
            case ECM_IF_STATE_BUSOFF:           return Master::ExtendedState::Busoff;
            case ECM_IF_STATE_LEAVE_OP:         return Master::ExtendedState::LeaveOp;
            case ECM_IF_STATE_BUSSCAN:          return Master::ExtendedState::Busscan;
            case ECM_IF_STATE_BUSSCAN_COMPLETE: return Master::ExtendedState::BusscanComplete;
            default:
                break;
        }

        std::stringstream ss;

        // If failed to convert, compose error message
        ss << "[" << context << "] "
           << "Invalid master state ("
           << std::to_string(static_cast<int>(state))
           << ")";
        // Throw error
        throw std::range_error { ss.str() };
        
    }

}


Master::StateInfo Master::get_state_info(std::chrono::milliseconds timeout) {

    // Zero-initialize request structure
    ECM_IF_GET_MASTER_CURRENT_STATE_REQ_T req { };

    // Fill request header
    req.tHead.ulDest = HIL_PACKET_DEST_DEFAULT_CHANNEL;
    req.tHead.ulLen  = 0;
    req.tHead.ulCmd  = ECM_IF_CMD_GET_MASTER_CURRENT_STATE_REQ;

    // Zero initialize response structure
    ECM_IF_GET_MASTER_CURRENT_STATE_CNF_T res { };

    // Exchange packet with the CIFX device
    channel.get_mailbox().exchange_packet(req, res, timeout);

    // Check if transfer aborted
    if(res.tHead.ulSta != SUCCESS_HIL_OK)
        throw cifx::Error{ CIFX_DEV_FUNCTION_FAILED, "cifx::ethercat::Master::get_state_info" };

    // Return current state
    return StateInfo {
        .target_state  = details::cifx_to_state(res.tData.bCurrentState, "cifx::ethercat::Master::get_state_info"),
        .current_state = details::cifx_to_state(res.tData.bTargetState,  "cifx::ethercat::Master::get_state_info"),
        .stop_reason   = res.tData.ulStopReason,
        .flags {
            .at_least_one_mandatory_slave_not_in_op = bool(res.tData.ulMasterStatusFlags & MSK_ECM_IF_MASTER_STATUS_FLAGS_AT_LEAST_ONE_MANDATORY_SLAVE_NOT_IN_OP),
            .dc_xrmw_stopped                        = bool(res.tData.ulMasterStatusFlags & MSK_ECM_IF_MASTER_STATUS_FLAGS_DC_XRMW_STOPPED                       ),
            .at_least_one_mandatory_slave_lost      = bool(res.tData.ulMasterStatusFlags & MSK_ECM_IF_MASTER_STATUS_FLAGS_AT_LEAST_ONE_MANDATORY_SLAVE_LOST     )
        }
    };
}

/* ================================================= Public CIFX-specific methods ================================================= */

namespace details {
namespace {

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

}}


void Master::set_sync_mode(SyncMode mode, std::chrono::milliseconds timeout) {

    details::SyncModeConfig target_config;
    
    // Select configuration to be set
    switch(mode) {
        case SyncMode::FreeRun: target_config = details::FreeRunSyncModeConfig; break;
        case SyncMode::IO1:     target_config = details::IO1SyncModeConfig;     break;
        case SyncMode::IO2:     target_config = details::IO2SyncModeConfig;     break;
        default:
            throw cifx::Error{ 
                CIFX_INVALID_PARAMETER,
                "[cifx::ethercat::Master::set_sync_mode] Invalid SyncMode given" 
            };
    }

    // Zero-initialize request structure
    HIL_SET_HANDSHAKE_CONFIG_REQ_T req { };

    // Fill request header
    req.tHead.ulDest = HIL_PACKET_DEST_DEFAULT_CHANNEL;
    req.tHead.ulLen  = sizeof(HIL_SET_HANDSHAKE_CONFIG_REQ_DATA_T);
    req.tHead.ulCmd  = HIL_SET_HANDSHAKE_CONFIG_REQ;
    // Prepare request's body
    req.tData.bPDInHskMode   = target_config.bPDInHskMode;
    req.tData.bPDInSource    = target_config.bPDInSource;
    req.tData.bPDOutHskMode  = target_config.bPDOutHskMode;
    req.tData.bPDOutSource   = target_config.bPDOutSource;

    // Zero initialize response structure
    HIL_SET_HANDSHAKE_CONFIG_CNF_T res { };

    // Exchange packet with the CIFX device
    channel.get_mailbox().exchange_packet(req, res, timeout);

    // Check if transfer aborted
    if(res.tHead.ulSta != SUCCESS_HIL_OK)
        throw cifx::Error{ CIFX_DEV_FUNCTION_FAILED, "cifx::ethercat::Master::set_sync_mode" };
}


Master::TimingInfo Master::get_timing_info(std::chrono::milliseconds timeout) {


    // Zero-initialize request structure
    ECM_IF_GET_TIMING_INFO_REQ_T req { };

    // Fill request header
    req.tHead.ulDest = HIL_PACKET_DEST_DEFAULT_CHANNEL;
    req.tHead.ulLen  = 0;
    req.tHead.ulCmd  = ECM_IF_CMD_GET_TIMING_INFO_REQ;

    // Zero initialize response structure
    ECM_IF_GET_TIMING_INFO_CNF_T res { };

    // Exchange packet with the CIFX device
    channel.get_mailbox().exchange_packet(req, res, timeout);

    // Check if transfer aborted
    if(res.tHead.ulSta != SUCCESS_HIL_OK)
        throw cifx::Error{ CIFX_DEV_FUNCTION_FAILED, "cifx::ethercat::Master::set_sync_mode" };

    // Return timing info
    return TimingInfo {
        .bus_cycle              { std::chrono::nanoseconds(res.tData.ulBusCycleTimeNs)       },
        .frame_transmition_time { std::chrono::nanoseconds(res.tData.ulFrameTransmitTimeNs)  },
        .expected_bus_delay     { std::chrono::nanoseconds(res.tData.ulExpectedBusDelayNs)   },
        .expected_rx_end_time   { std::chrono::nanoseconds(res.tData.ulExpectedRxEndTimeNs)  },
        .expected_tx_end_time   { std::chrono::nanoseconds(res.tData.ulExpectedTxDataTimeNs) }
    };
}

/* ========================================== Protected common methods (implementations) ========================================== */

namespace details {

    static inline uint8_t state_to_cifx(Master::State state, std::string_view context) {

        // Convert the state
        switch(state) {
            case Master::State::Init:   return ECM_IF_STATE_INIT;
            case Master::State::Preop:  return ECM_IF_STATE_PREOP;
            case Master::State::Safeop: return ECM_IF_STATE_SAFEOP;
            case Master::State::Op:     return ECM_IF_STATE_OP;
            default:
                break;
        }

        std::stringstream ss;

        // If failed to convert, compose error message
        ss << "[" << context << "] "
           << "Invalid master state ("
           << std::to_string(static_cast<int>(state))
           << ")";
        // Throw error
        throw std::range_error { ss.str() };
     
    }

}


void Master::set_state_impl(State state, std::chrono::milliseconds timeout) {

    // Zero-initialize request structure
    ECM_IF_SET_MASTER_TARGET_STATE_REQ_T req { };

    // Fill request header
    req.tHead.ulDest = HIL_PACKET_DEST_DEFAULT_CHANNEL;
    req.tHead.ulLen  = sizeof(ECM_IF_SET_MASTER_TARGET_STATE_REQ_DATA_T);
    req.tHead.ulCmd  = ECM_IF_CMD_SET_MASTER_TARGET_STATE_REQ;
    // Fill request header
    req.tData.bTargetState = details::state_to_cifx(state, "cifx::ethercat::Master::set_state_impl");

    // Zero initialize response structure
    ECM_IF_SET_MASTER_TARGET_STATE_CNF_T res { };

    // Exchange packet with the CIFX device
    channel.get_mailbox().exchange_packet(req, res, timeout);

    // Check if transfer aborted
    if(res.tHead.ulSta != SUCCESS_HIL_OK)
        throw cifx::Error{ CIFX_DEV_FUNCTION_FAILED, "cifx::ethercat::Master::set_state_impl" };
        
}

/* ================================================================================================================================ */

} // End namespace cifx::ethercat
