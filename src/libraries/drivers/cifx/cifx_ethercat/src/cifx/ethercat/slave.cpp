/* ============================================================================================================================ *//**
 * @file       slave.cpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 18th May 2022 10:31:09 am
 * @modified   Friday, 1st July 2022 2:26:48 pm
 * @project    engineering-thesis
 * @brief      Definition of methods of the Slave class representing slave device on the EtherCAT bus
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

/* =========================================================== Includes =========================================================== */

// CIFX includes
#include "Hil_ApplicationCmd.h"
#include "Hil_Results.h"
// Private includes
#include "cifx/ethercat/slave.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace cifx::ethercat {

/* =============================================== Private implementation functions =============================================== */

namespace details {

    static inline Slave::State cifx_to_state(uint8_t state, std::string_view context) {

        // Convert the state
        switch(state) {
            case ECM_IF_STATE_INIT:   return Slave::State::Init;
            case ECM_IF_STATE_PREOP:  return Slave::State::Preop;
            case ECM_IF_STATE_BOOT:   return Slave::State::Boot;
            case ECM_IF_STATE_SAFEOP: return Slave::State::Safeop;
            case ECM_IF_STATE_OP:     return Slave::State::Op;
            default: 
                break;
        }

        std::stringstream ss;

        // If failed to convert, compose error message
        ss << "[" << context << "] "
           << "Invalid slave state ("
           << std::to_string(static_cast<int>(state))
           << ")";
        // Throw error
        throw std::range_error { ss.str() };
    }

    static inline Slave::ExtendedState cifx_to_extended_state(uint8_t state, std::string_view context) {

        // Convert the state
        switch(state) {
            case ECM_IF_STATE_BUSOFF:     return Slave::ExtendedState::Busoff;
            case ECM_IF_STATE_INIT:       return Slave::ExtendedState::Init;
            case ECM_IF_STATE_PREOP:      return Slave::ExtendedState::Preop;
            case ECM_IF_STATE_BOOT:       return Slave::ExtendedState::Boot;
            case ECM_IF_STATE_SAFEOP:     return Slave::ExtendedState::Safeop;
            case ECM_IF_STATE_OP:         return Slave::ExtendedState::Op;
            case ECM_IF_STATE_INIT_ERR:   return Slave::ExtendedState::InitErr;
            case ECM_IF_STATE_PREOP_ERR:  return Slave::ExtendedState::PreopErr;
            case ECM_IF_STATE_BOOT_ERR:   return Slave::ExtendedState::BootErr;
            case ECM_IF_STATE_SAFEOP_ERR: return Slave::ExtendedState::SafeopErr;
            default: 
                break;
        }

        std::stringstream ss;

        // If failed to convert, compose error message
        ss << "[" << context << "] "
           << "Invalid slave state ("
           << std::to_string(static_cast<int>(state))
           << ")";
        // Throw error
        throw std::range_error { ss.str() };
    }

    static inline uint8_t state_to_cifx(Slave::State state, std::string_view context) {

        // Convert the state
        switch(state) {
            case Slave::State::Init:   return ECM_IF_STATE_INIT;
            case Slave::State::Preop:  return ECM_IF_STATE_PREOP;
            case Slave::State::Boot:   return ECM_IF_STATE_BOOT;
            case Slave::State::Safeop: return ECM_IF_STATE_SAFEOP;
            case Slave::State::Op:     return ECM_IF_STATE_OP;
            default: 
                break;
        }

        std::stringstream ss;

        // If failed to convert, compose error message
        ss << "[" << context << "] "
           << "Invalid slave state ("
           << std::to_string(::ethercat::common::utilities::to_underlying(state))
           << ")";
        // Throw error
        throw std::range_error { ss.str() };
    }

}

/* ===================================================== Public common methods ==================================================== */

Slave::StateInfo Slave::get_state_info(std::chrono::milliseconds timeout) const {

    // Zero-initialize request structure
    ECM_IF_GET_SLAVE_CURRENT_STATE_REQ_T req { };

    // Fill request header
    req.tHead.ulDest = HIL_PACKET_DEST_DEFAULT_CHANNEL;
    req.tHead.ulLen  = sizeof(ECM_IF_GET_SLAVE_CURRENT_STATE_REQ_DATA_T);
    req.tHead.ulCmd  = ECM_IF_CMD_GET_SLAVE_CURRENT_STATE_REQ;
    // Fill request header
    req.tData.usStationAddress = fixed_addr;

    // Zero initialize response structure
    ECM_IF_GET_SLAVE_CURRENT_STATE_CNF_T res { };

    // Exchange packet with the CIFX device
    channel.get_mailbox().exchange_packet(req, res, timeout);

    // Check if transfer aborted
    if(res.tHead.ulSta != SUCCESS_HIL_OK)
        throw cifx::Error{ CIFX_DEV_FUNCTION_FAILED, "cifx::ethercat::Slave::get_state_info" };
        
    // Return current state
    return StateInfo {
        .current_state = details::cifx_to_extended_state(res.tData.bCurrentState, "cifx::ethercat::Slave::get_state_info"),
        .target_state  = details::cifx_to_state         (res.tData.bTargetState,  "cifx::ethercat::Slave::get_state_info"),
        .error         = bool(res.tData.ulActiveError)
    };
}

/* ================================================== Public SDO-related methods ================================================== */

std::vector<uint16_t> Slave::get_sdo_list(SDOListInfoMode mode, std::chrono::milliseconds timeout) {

    /**
     * @see 'EtherCAT Master V4 Protocol API 06 EN.pdf', p.86-87
     */

    std::vector<uint16_t> ret;

    // Zero-initialize request structure
    ECM_IF_COE_SDOINFO_GETODLIST_REQ_T req { };

    // Fill request header
    req.tHead.ulDest   = HIL_PACKET_DEST_DEFAULT_CHANNEL;
    req.tHead.ulDestId = 0U;
    req.tHead.ulLen    = sizeof(ECM_IF_COE_SDOINFO_GETODLIST_REQ_DATA_T);
    req.tHead.ulCmd    = ECM_IF_CMD_COE_SDOINFO_GETODLIST_REQ;
    // Fill request data
    req.tData.usStationAddress = fixed_addr;
    req.tData.usTransportType  = ECM_IF_COE_TRANSPORT_COE;
    req.tData.usListType       = static_cast<decltype(req.tData.usListType)>(mode);
    req.tData.ulTimeoutMs      = static_cast<decltype(req.tData.ulTimeoutMs)>(timeout.count());
    req.tData.ulMaxTotalBytes  = 10240; // Set some reasonable size of the output buffer

    // Zero initialize response structure
    ECM_IF_COE_SDOINFO_GETODLIST_CNF_T res { };

    // Number of data bytes that can be rad from the CIFX device in one exchange
    constexpr std::size_t PACKET_DATA_BUFFER_SIZE = sizeof(res.tData.ausObjectIDs);

    // Initialize number of data bytes already received
    std::size_t bytes_received = 0U;        

    // Context of potential error messages
    constexpr auto context = "cifx::ethercat::Slave::get_sdo_list";
    
    // Exchange data with the CIFX device untill full object data is passed
    while(true) {

        // Set packet sequence index depending on whether middle or last package is to be exchanged
        req.tHead.ulExt = (bytes_received == 0U) ? HIL_PACKET_SEQ_NONE : HIL_PACKET_SEQ_MIDDLE;
        // Exchange packet
        try {
            channel.get_mailbox().exchange_partial_packet(
                req,
                res,
                timeout
            );
        } catch(cifx::Error &err) {
            err.rethrow_with_context(context);
        }

        // Parse total number of bytes received
        std::size_t total_bytes = res.tData.ulTotalBytes;

        // If first package has been exchanged, prepare output buffer absed on the size of info structure returned in the response
        if(bytes_received == 0U)
            ret.resize(total_bytes / sizeof(uint16_t));
        
        // Calculate amount of data received in the response
        std::size_t bytes_received_in_package = (bytes_received + PACKET_DATA_BUFFER_SIZE <= total_bytes) ?
            PACKET_DATA_BUFFER_SIZE :
            (total_bytes - bytes_received);
        // Copy chunk of data into the request
        std::copy_n(
            reinterpret_cast<uint8_t*>(res.tData.ausObjectIDs),
            bytes_received_in_package,
            reinterpret_cast<uint8_t*>(ret.data()) + bytes_received);
        // Update number of bytes sent
        bytes_received += bytes_received_in_package;

        // Make sure that the CIFX device returned valid sequence flag if all bytes has been received
        if((bytes_received == total_bytes) and (res.tHead.ulExt != HIL_PACKET_SEQ_NONE and res.tHead.ulExt != HIL_PACKET_SEQ_LAST))
            throw cifx::Error{ CIFX_DEV_FUNCTION_FAILED, context, "All bytes has been received, but CIFX device reports more data to be proceeded" };
        // Make sure that the CIFX device returned valid sequence flag if there are some bytes to be received yet
        if((bytes_received != total_bytes) and (res.tHead.ulExt == HIL_PACKET_SEQ_NONE or res.tHead.ulExt == HIL_PACKET_SEQ_LAST))
            throw cifx::Error{ CIFX_DEV_FUNCTION_FAILED, context, "There are bytes to be received yet, but CIFX device reports end of proceeding" };

        // If all bytes received, quit transfer
        if(bytes_received == total_bytes)
            break;
    }

    return ret;
}

/* ========================================== Protected common methods (implementations) ========================================== */

Slave::State Slave::get_state_impl(std::chrono::milliseconds timeout) const {

    // Get slave's state info
    auto current_state = get_state_info(timeout).current_state;
    // Map CIFX-specific state to the EtherCAT-conformant state
    switch(current_state) {
        case ExtendedState::Busoff:    return State::Init;
        case ExtendedState::Init:      return State::Init;
        case ExtendedState::Preop:     return State::Preop;
        case ExtendedState::Boot:      return State::Boot;
        case ExtendedState::Safeop:    return State::Safeop;
        case ExtendedState::Op:        return State::Op;
        case ExtendedState::InitErr:   return State::Init;
        case ExtendedState::PreopErr:  return State::Preop;
        case ExtendedState::BootErr:   return State::Boot;
        case ExtendedState::SafeopErr: return State::Safeop;
        default: /* Should not happen */
            break;
    }

    using namespace std::literals::string_literals;

    // Throw unexpected error
    throw std::runtime_error{ 
        "[cifx::ethercat::Slave::get_state_impl] Unexpected Slave state read from the device " 
          "("s
        + std::to_string(static_cast<std::underlying_type_t<ExtendedState>>(current_state))
        + ")"
    };
}


void Slave::set_state_impl(State state, std::chrono::milliseconds timeout) {

    // Zero-initialize request structure
    ECM_IF_SET_SLAVE_TARGET_STATE_REQ_T req { };

    // Fill request header
    req.tHead.ulDest = HIL_PACKET_DEST_DEFAULT_CHANNEL;
    req.tHead.ulLen  = sizeof(ECM_IF_SET_SLAVE_TARGET_STATE_REQ_DATA_T);
    req.tHead.ulCmd  = ECM_IF_CMD_SET_SLAVE_TARGET_STATE_REQ;
    // Fill request header
    req.tData.usStationAddress = fixed_addr;
    req.tData.bTargetState     = details::state_to_cifx(state, "cifx::ethercat::Slave::set_state");

    // Zero initialize response structure
    ECM_IF_SET_SLAVE_TARGET_STATE_CNF_T res { };

    // Exchange packet with the CIFX device
    channel.get_mailbox().exchange_packet(req, res, timeout);

    // Check if transfer aborted
    if(res.tHead.ulSta != SUCCESS_HIL_OK)
        throw cifx::Error{ CIFX_DEV_FUNCTION_FAILED, "cifx::ethercat::Slave::set_state" };
}

/* ============================================= Private implementation methods (SDO) ============================================= */

/**
 * @addtogroup private
 * @{
 */

namespace details {
    
    template<bool flag = false>
    constexpr inline void initialize_sdo_request_packet_dir_no_match() {
        static_assert(flag, "[cifx::ethercat::Slave::initialize_common_sdo_request][BUG] Invalid SDO direction given"); 
    }
    
    /**
     * @brief Initializes part of the SDO download/upload request common to both
     *    directions
     * 
     * @tparam dir 
     *    direction of the SDO
     * @tparam RequestStructT 
     *    type of the request structure
     * @param index 
     *    index of the object
     * @param subindex 
     *    subindex of the object
     * @param complete_access 
     *    @c true if complete access is to be performed
     * @param req 
     *    request to be initialized
     * @param fixed_addr 
     *    fixed address of the slave
     * @param data 
     *    reference to the data/data buffer associated with the request
     * @param initial_timeout 
     *    initial timeout
     */
    template<Slave::SdoDirection dir, typename RequestStructT>
    static inline void initialize_common_sdo_request(
        uint16_t index,
        uint16_t subindex,
        bool complete_access,
        RequestStructT &req,
        uint16_t fixed_addr,
        ranges::span<const uint8_t> data,
        std::chrono::milliseconds initial_timeout
    ) {
        // Fill request header
        req.tHead.ulDest   = HIL_PACKET_DEST_DEFAULT_CHANNEL;
        req.tHead.ulDestId = 0U;
        // Fill request data
        req.tData.usStationAddress = fixed_addr;
        req.tData.usTransportType  = ECM_IF_COE_TRANSPORT_COE;
        req.tData.usObjIndex       = index;
        req.tData.bSubIndex        = subindex;
        req.tData.fCompleteAccess  = complete_access;
        req.tData.ulTimeoutMs      = static_cast<decltype(req.tData.ulTimeoutMs)>(initial_timeout.count());
        // Take into account naming variance in case of different directions
        if constexpr (dir == Slave::SdoDirection::Download)
            req.tData.ulTotalBytes = static_cast<decltype(req.tData.ulTotalBytes)>(data.size());
        else if constexpr (dir == Slave::SdoDirection::Upload)
            req.tData.ulMaxTotalBytes = static_cast<decltype(req.tData.ulMaxTotalBytes)>(data.size());
        else 
            initialize_sdo_request_packet_dir_no_match();
    }

    /**
     * @param transfered_bytes 
     *    number of bytes alredy transfered
     * @param target_bytes 
     *    number of bytes to transfer
     * @param chunk_bytes 
     *    max number of bytes transfered in the single echange
     * @returns 
     *    value of the ulExt field of the header packet requesting SDO download to the slave
     */
    static inline uint32_t get_download_packet_sequence_id(
        std::size_t transfered_bytes,
        std::size_t target_bytes,
        std::size_t chunk_bytes
    ) {
        if(transfered_bytes == 0U && target_bytes <= chunk_bytes)
            return HIL_PACKET_SEQ_NONE;
        else if(transfered_bytes == 0U)
            return HIL_PACKET_SEQ_FIRST;
        else if(target_bytes - transfered_bytes > chunk_bytes)
            return HIL_PACKET_SEQ_MIDDLE;
        else
            return HIL_PACKET_SEQ_LAST;
    }

}

/**
 * @}
 */

void Slave::download_sdo(
    uint16_t index,
    uint16_t subindex,
    ::ethercat::config::types::Span<const uint8_t> data,
    std::chrono::milliseconds timeout,
    bool complete_access
) {

    /**
     * @see 'EtherCAT Master V4 Protocol API 06 EN.pdf', p.70-79
     */

    // Zero-initialize request structure
    ECM_IF_COE_SDO_DOWNLOAD_REQ_T req { };

    // Fill common part of the request
    details::initialize_common_sdo_request<SdoDirection::Download>(
        index,
        subindex,
        complete_access,
        req,
        fixed_addr,
        data,
        timeout);
    // Fill direction-specific fields
    req.tHead.ulLen = sizeof(ECM_IF_COE_SDO_DOWNLOAD_REQ_DATA_T);
    req.tHead.ulCmd = ECM_IF_CMD_COE_SDO_DOWNLOAD_REQ;

    // Zero initialize response structure
    ECM_IF_COE_SDO_DOWNLOAD_CNF_T res { };

    // Number of data bytes that can be sent to the CIFX device in one request
    constexpr std::size_t PACKET_DATA_BUFFER_SIZE = std::size(req.tData.abData);
    // Context string of the method
    constexpr auto context = "cifx::ethercat::Slave::download_sdo";
        
    // Initialize number of data bytes already sent
    std::size_t bytes_sent = 0U;
    // Initialize number of data bytes to be sent
    const std::size_t bytes_to_send = data.size();        
    
    // Exchange data with the CIFX device untill full object data is passed
    while(bytes_sent < bytes_to_send) {

        // Set packet sequence index depending on thwther middle or last package is to be exchanged
        req.tHead.ulExt = details::get_download_packet_sequence_id(bytes_sent, bytes_to_send, PACKET_DATA_BUFFER_SIZE);
        // Calculate size of data to be put into the request
        std::size_t data_to_send_in_next_packet = std::min(bytes_to_send - bytes_sent, PACKET_DATA_BUFFER_SIZE);
        // Copy chunk of data into the request
        std::copy_n(data.begin() + bytes_sent, data_to_send_in_next_packet, req.tData.abData);

        /**
         * @note In opposite to all other packages (for some reason) in case of the SDO download request, size of the
         *    request given in the packet's header needs to be ACTUAL number of bytes to be processed by the CIFX device,
         *    NOT size of the ECM_IF_COE_SDO_DOWNLOAD_REQ_DATA_T structure.
         */
        req.tHead.ulLen = 
              sizeof(ECM_IF_COE_SDO_DOWNLOAD_REQ_DATA_T)
            - sizeof(ECM_IF_COE_SDO_DOWNLOAD_REQ_DATA_T::abData)
            + data_to_send_in_next_packet;

        // Exchange packet
        try {
            channel.get_mailbox().exchange_partial_packet(
                req,
                res,
                timeout
            );
        } catch(cifx::Error &err) {
            err.rethrow_with_context(context);
        }
        
        // Update number of bytes sent
        bytes_sent += data_to_send_in_next_packet;
    }
}


void Slave::upload_sdo(
    uint16_t index,
    uint16_t subindex,
    ::ethercat::config::types::Span<uint8_t> data,
    std::chrono::milliseconds timeout,
    bool complete_access
) const {

    /**
     * @see 'EtherCAT Master V4 Protocol API 06 EN.pdf', p.70-79
     */

    // Zero-initialize request structure
    ECM_IF_COE_SDO_UPLOAD_REQ_T req { };

    // Fill common part of the request
    details::initialize_common_sdo_request<SdoDirection::Upload>(
        index,
        subindex,
        complete_access,
        req,
        fixed_addr,
        data,
        timeout);
    // Fill direction-specific fields
    req.tHead.ulLen = sizeof(ECM_IF_COE_SDO_UPLOAD_REQ_DATA_T);
    req.tHead.ulCmd = ECM_IF_CMD_COE_SDO_UPLOAD_REQ;

    // Zero initialize response structure
    ECM_IF_COE_SDO_UPLOAD_CNF_T res { };

    // Number of data bytes that can be rad from the CIFX device in one exchange
    constexpr std::size_t PACKET_DATA_BUFFER_SIZE = std::size(res.tData.abData);
    // Context of potential error messages
    constexpr auto context = "cifx::ethercat::Slave::upload_sdo";

    // Initialize number of data bytes already received
    std::size_t bytes_received = 0U;
    // Initialize number of data bytes to be received
    const std::size_t bytes_to_receive = data.size();

    // Exchange data with the CIFX device untill full object data is passed
    while(bytes_received < bytes_to_receive) {

        // Set packet sequence index depending on thwther middle or last package is to be exchanged
        req.tHead.ulExt = (bytes_received == 0U) ? HIL_PACKET_SEQ_NONE : HIL_PACKET_SEQ_MIDDLE;
        // Calculate size of data to be read from the response
        std::size_t data_to_receive_from_next_packet = std::min(bytes_to_receive - bytes_received, PACKET_DATA_BUFFER_SIZE);

        // Exchange packet
        try {
            channel.get_mailbox().exchange_partial_packet(
                req,
                res,
                timeout
            );
        } catch(cifx::Error &err) {
            err.rethrow_with_context(context);
        }
        
        // Copy chunk of data into the request
        std::copy_n(res.tData.abData, data_to_receive_from_next_packet, data.begin() + bytes_received);
        // Update number of bytes sent
        bytes_received += data_to_receive_from_next_packet;

        // Make sure that the CIFX device returned valid sequence flag if all bytes has been received
        if((bytes_received == bytes_to_receive) and (res.tHead.ulExt != HIL_PACKET_SEQ_NONE and res.tHead.ulExt != HIL_PACKET_SEQ_LAST))
            throw cifx::Error{ CIFX_DEV_FUNCTION_FAILED, context, "All bytes has been received, but CIFX device reports more data to be proceeded" };
        // Make sure that the CIFX device returned valid sequence flag if there are some bytes to be received yet
        if((bytes_received != bytes_to_receive) and (res.tHead.ulExt == HIL_PACKET_SEQ_NONE or res.tHead.ulExt == HIL_PACKET_SEQ_LAST))
            throw cifx::Error{ CIFX_DEV_FUNCTION_FAILED, context, "There are bytes to be received yet, but CIFX device reports end of proceeded" };
    }
}

/* ================================================================================================================================ */

} // End namespace cifx::ethercat
