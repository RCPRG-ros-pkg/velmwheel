/* ============================================================================================================================ *//**
 * @file       master.cpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 26th May 2022 10:25:27 pm
 * @modified   Friday, 1st July 2022 12:39:10 pm
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

/* ===================================================== Public ctors & dtors ===================================================== */

Master::Master(cifx::Channel &channel) :

    // Base interface
    MasterInterfaceT{ 

        // Get path to the ENI file
        channel.get_device().get_config_file(),
        // Pass implementation-specific factory functory creating instances of Slave device interface
        [&channel](
            ::ethercat::eni::Slave slave_eni,
            std::vector<::ethercat::Slave<Slave>::Pdo<::ethercat::Slave<Slave>::PdoDirection::Input>> &&inputs,
            std::vector<::ethercat::Slave<Slave>::Pdo<::ethercat::Slave<Slave>::PdoDirection::Output>> &&outputs
        ) {
            return Slave{ 
                channel,
                slave_eni,
                std::move(inputs),
                std::move(outputs)
            };
        }

    },
    // Implementation-specific data
    channel{ channel }
    
{
    /// Parse ENI file
    auto eni_config = ::ethercat::eni::configruation_from_file(channel.get_device().get_config_file());
    // Request CIFX device to list mappings in the Communication I/O Areas
    auto mappings = get_cyclic_mapping_info();    

    auto compute_process_data_offset = [](
              auto &mapping_info,
        const auto &pdi_variables
    ) -> std::size_t {

        // Compute Process Data offset 
        if(not pdi_variables.empty()) {

            // Filter input mapping to contain only information about ProcessData mappings
            mapping_info.erase(std::remove_if(
                mapping_info.begin(), mapping_info.end(),
                [](const auto &m) { 
                    if constexpr(std::is_same_v<decltype(m.type), CyclicMappingInfo::RxEntry::Type>)
                        return (m.type != CyclicMappingInfo::RxEntry::Type::ProcessData );
                    else
                        return (m.type != CyclicMappingInfo::TxEntry::Type::ProcessData );
                }
            ), mapping_info.end());

            // Find input mapping holding processData with the lowest address
            auto first_pd_mapping = 
                std::min_element(mapping_info.begin(), mapping_info.end(),
                [](const auto &a, const auto &b){ return (a.offset < b.offset); }
            );

            // Calculate offset of the first variable mapped in the Process Data Image
            auto first_var = 
                std::min_element(pdi_variables.begin(), pdi_variables.end(),
                [](const auto &a, const auto &b){ return (a.get_byte_offset() < b.get_byte_offset()); }
            );
            
            constexpr std::size_t BITS_IN_BYTE = 8;
            
            /*
            * The following code assumes that the first variable in the Process Data Image is byte-aligned.
            * This is always the case, if ENI configruation has been generated with the TwinCAT  software
            * but it doesn't have to be true for ENI in general
            */            
            
            assert(first_var->get_bit_offset() % BITS_IN_BYTE == 0);

            /// Calculate offset that needs to be added to the CIFX API calls when reading Process Data Image buffers
            return (first_pd_mapping->offset  - first_var->get_byte_offset());
            
        }
        
        // By default return 0 offset
        return 0;
    };

    // Compute Process Data offset for Input direction
    input_pdi_data_offset = 
        compute_process_data_offset(
            mappings.rx,
            eni_config.get_process_image().get_variables().inputs
        );

    // Compute Process Data offset for Output direction
    output_pdi_data_offset = 
        compute_process_data_offset(
            mappings.tx,
            eni_config.get_process_image().get_variables().outputs
        );

}

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


Master::StateInfo Master::get_state_info(std::chrono::milliseconds timeout) const {

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

/* ========================================== Public CIFX-specific methods (bus control) ========================================== */

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


Master::TimingInfo Master::get_timing_info(std::chrono::milliseconds timeout) const {


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

/* ================================================= Common implementation helpers ================================================ */

namespace details {

    /**
     * @brief Auxiliary function exchangin packet with the CIFX device updating @p timeout 
     *    value with the time consumed by the exchange. Updates ulDestId @p req header
     *    with the value read from the succesfull response
     * 
     * @tparam RequestT 
     *    type of the request structure
     * @tparam ResponseT 
     *    type of the response structure
     * @param channel 
     *    CIFX channel to be used to communicate with device
     * @param[inout] req 
     *    request to be sent
     * @param[out] res 
     *    response buffer
     * @param timeout 
     *    exchange timeout
     * @param context 
     *    contextual string for error message
     */
    template<typename RequestT, typename ResponseT>
    static inline void exchange_packet_with_timeout_update(
        cifx::Channel &channel,
        RequestT &req,
        ResponseT &res,
        std::chrono::milliseconds &timeout,
        std::string_view context
    ) {

        using namespace std::literals::chrono_literals;

        // Denote start time of the routine
        auto start_time_point = std::chrono::steady_clock::now();

        // Exchange packet with the CIFX device
        channel.get_mailbox().exchange_packet(req, res, timeout);
        
        // Check if transfer aborted
        if(res.tHead.ulSta != SUCCESS_HIL_OK) {
            std::stringstream ss;
            ss << "CIFX device reported error [ulSta: 0x" << std::hex << std::uppercase << res.tHead.ulSta << "]";
            throw cifx::Error{ CIFX_DEV_FUNCTION_FAILED, context, ss.str() };
        }

        // Update timeout left
        timeout -= std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time_point);
        // If timeout reached, throw error
        if(timeout <= 0ms)
            throw cifx::Error{ CIFX_DEV_EXCHANGE_TIMEOUT, context, "Exchange timeouted" };

        // Set packet's destination ID for the next request the one returned by the CIFX device in the response
        req.tHead.ulDestId = res.tHead.ulDestId;
    }

}

/* ======================================== Public CIFX-specific methods (comm-area mapping) ====================================== */

std::string Master::CyclicMappingInfo::to_str() const {

    std::stringstream ss;

    // Print header
    ss << "CIFX EtherCAT mappings:" << std::endl;

    // Print header of the input mappings
    ss << "  --> Input Area: " << std::endl;
    // Serialize inptu mappings
    for(auto &m : rx) {
        ss << "     - type:   " << CyclicMappingInfo::RxEntry::type_to_str(m.type) << std::endl;
        ss << "       offset: " << m.offset << " [B]"                              << std::endl;
        ss << "       size:   " << m.size   << " [B]"                              << std::endl;
    }

    // Print header of the output mappings
    ss << "   --> Output Area: " << std::endl;
    // Serialize output mappings
    for(auto &m : tx) {
        ss << "     - type:   " << CyclicMappingInfo::TxEntry::type_to_str(m.type) << std::endl;
        ss << "       offset: " << m.offset << " [B]"                              << std::endl;
        ss << "       size:   " << m.size   << " [B]"                              << std::endl;
    }

    // Return result
    return ss.str();
}

Master::CyclicMappingInfo Master::get_cyclic_mapping_info(std::chrono::milliseconds timeout) const {

    CyclicMappingInfo ret;

    // Zero-initialize request structure
    ECM_IF_GET_CYCLIC_CMD_MAPPING_REQ_T req { };

    // Fill request header
    req.tHead.ulDest   = HIL_PACKET_DEST_DEFAULT_CHANNEL;
    req.tHead.ulDestId = 0U;
    req.tHead.ulLen    = sizeof(ECM_IF_GET_CYCLIC_CMD_MAPPING_REQ_DATA_T);
    req.tHead.ulCmd    = ECM_IF_CMD_GET_CYCLIC_CMD_MAPPING_REQ;

    // Zero initialize response structure
    ECM_IF_GET_CYCLIC_CMD_MAPPING_CNF_T res { };

    // Number of data bytes that can be rad from the CIFX device in one exchange
    constexpr std::size_t ENTRIES_IN_PACKET_MAX = sizeof(res.tData.atEntries) / sizeof(ECM_IF_CYCLIC_CMD_MAPPING_ENTRY_T);
    // Context of potential error messages
    constexpr auto context = "cifx::ethercat::Master::get_cyclic_mapping_info";

    // Initialize number of data entries already received
    std::size_t entries_received = 0U;
    
    // Exchange data with the CIFX device untill full object data is passed
    while(true) {

        // Fill request data
        req.tData.ulEntriesStartOffset = entries_received;
        // Exchange packet
        try {
            channel.get_mailbox().exchange_packet_with_timeout_update(
                req,
                res,
                timeout
            );
        } catch(cifx::Error &err) {
            err.rethrow_with_context(context);
        }

        // If first package has been exchanged, prepare memory for output buffers 
        if(entries_received == 0U) {
            ret.rx.reserve(res.tData.ulTotalEntries);
            ret.tx.reserve(res.tData.ulTotalEntries);
        }
        
        // Calculate amount of data received in the response
        std::size_t entries_received_in_package = (entries_received + ENTRIES_IN_PACKET_MAX <= res.tData.ulTotalEntries) ?
            ENTRIES_IN_PACKET_MAX :
            (res.tData.ulTotalEntries - entries_received);
        // Update number of entries received
        entries_received += entries_received_in_package;

        // Parse entries
        for(std::size_t i = 0; i < entries_received_in_package; ++i) {

            // Parse packed field to meet std::optional constructor requirements
            uint16_t wck_compare_received_byte_offset = res.tData.atEntries[i].usWkcCompareReceiveByteOffset;

            // Parse RX Entry
            ret.rx.push_back(
                CyclicMappingInfo::RxEntry {

                    // Parse basic info
                    .type   = static_cast<CyclicMappingInfo::RxEntry::Type>(res.tData.atEntries[i].usReceiveType),
                    .offset = res.tData.atEntries[i].usRxImageStartByteOffset,
                    .size   = res.tData.atEntries[i].usImageByteLength,
                    // Parse WKC info
                    .wkc_compare_received_byte_offset = 
                        (wck_compare_received_byte_offset == 0xFFFF) ? 
                            std::optional<uint16_t>{ } : wck_compare_received_byte_offset

                }
            );

            // Parse TX Entry
            ret.tx.push_back(
                CyclicMappingInfo::TxEntry {

                    // Parse basic info
                    .type   = static_cast<CyclicMappingInfo::TxEntry::Type>(res.tData.atEntries[i].usTransmitType),
                    .offset = res.tData.atEntries[i].usTxImageStartByteOffset,
                    .size   = res.tData.atEntries[i].usImageByteLength,
                    
                }
            );

        }
        
        // If all bytes received, quit transfer
        if(entries_received == res.tData.ulTotalEntries)
            break;
    }

    return ret;
}


/* ====================================== Protected EtherCAT common methods (implementations) ===================================== */

Master::State Master::get_state_impl(std::chrono::milliseconds timeout) const {

    // Get master's state info
    auto current_state = get_state_info(timeout).current_state;
    // Map CIFX-specific state to the EtherCAT-conformant state
    switch(current_state) {
        case ExtendedState::Init:            return State::Init;
        case ExtendedState::Preop:           return State::Preop;
        case ExtendedState::Safeop:          return State::Safeop;
        case ExtendedState::Op:              return State::Op;
        case ExtendedState::Busoff:          return State::Init;
        case ExtendedState::LeaveOp:         return State::Op;
        case ExtendedState::Busscan:         return State::Init;
        case ExtendedState::BusscanComplete: return State::Init;
        default: /* Should not happen */
            break;
    }

    using namespace std::literals::string_literals;

    // Throw unexpected error
    throw std::runtime_error{ 
        "[cifx::ethercat::Master::get_state_impl] Unexpected Master state read from the device " 
          "("s
        + std::to_string(static_cast<std::underlying_type_t<ExtendedState>>(current_state))
        + ")"
    };
}

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
