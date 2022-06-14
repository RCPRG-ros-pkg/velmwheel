/* ============================================================================================================================ *//**
 * @file       slave.cpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 18th May 2022 10:31:09 am
 * @modified   Monday, 13th June 2022 11:31:40 am
 * @project    engineering-thesis
 * @brief      Definition of methods of the Slave class representing slave device on the EtherCAT bus
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
#include "cifx/ethercat/slave.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace cifx::ethercat {

/* ===================================================== Public common methods ==================================================== */

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

}

Slave::StateInfo Slave::get_state_info(std::chrono::milliseconds timeout) {

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
        .current_state = details::cifx_to_state(res.tData.bCurrentState, "cifx::ethercat::Slave::get_state_info"),
        .target_state  = details::cifx_to_state(res.tData.bTargetState,  "cifx::ethercat::Slave::get_state_info"),
        .error         = bool(res.tData.ulActiveError)
    };
}

/* ========================================== Protected common methods (implementations) ========================================== */

namespace details {

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
        req.tData.ulTimeoutMs = static_cast<decltype(req.tData.ulTimeoutMs)>(initial_timeout.count());
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
        if(res.tHead.ulSta != SUCCESS_HIL_OK)
            throw cifx::Error{ CIFX_DEV_FUNCTION_FAILED, context, "CIFX device reported error" };

        // Update timeout left
        timeout -= std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time_point);
        // If timeout reached, throw error
        if(timeout <= 0ms)
            throw cifx::Error{ CIFX_DEV_EXCHANGE_TIMEOUT, context, "Exchange timeouted" };

        // Set packet's destination ID for the next request the one returned by the CIFX device in the response
        req.tHead.ulDestId = res.tHead.ulDestId;
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

        // Exchange packet
        details::exchange_packet_with_timeout_update(
            channel,
            req,
            res,
            timeout,
            "cifx::ethercat::Slave::download_sdo"
        );
        
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
) {

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

    // Initialize number of data bytes already received
    std::size_t bytes_received = 0U;
    // Initialize number of data bytes to be received
    const std::size_t bytes_to_receive = data.size();        

    // Context of potential error messages
    constexpr auto context = "cifx::ethercat::Slave::upload_sdo";

    // Exchange data with the CIFX device untill full object data is passed
    while(bytes_received < bytes_to_receive) {

        // Set packet sequence index depending on thwther middle or last package is to be exchanged
        req.tHead.ulExt = (bytes_received == 0U) ? HIL_PACKET_SEQ_NONE : HIL_PACKET_SEQ_MIDDLE;
        // Calculate size of data to be read from the response
        std::size_t data_to_receive_from_next_packet = std::min(bytes_to_receive - bytes_received, PACKET_DATA_BUFFER_SIZE);

        // Exchange packet
        details::exchange_packet_with_timeout_update(
            channel,
            req,
            res,
            timeout,
            context
        );
        
        // Copy chunk of data into the request
        std::copy_n(res.tData.abData, data_to_receive_from_next_packet, data.begin() + bytes_received);
        // Update number of bytes sent
        bytes_received += data_to_receive_from_next_packet;

        // Make sure that the CIFX device returned valid sequence flag if all bytes has been received
        if((bytes_received == bytes_to_receive) and (res.tHead.ulExt != HIL_PACKET_SEQ_NONE or res.tHead.ulExt != HIL_PACKET_SEQ_LAST))
            throw cifx::Error{ CIFX_DEV_FUNCTION_FAILED, context, "All bytes has been received, but CIFX device reports more data to be proceeded" };
        // Make sure that the CIFX device returned valid sequence flag if there are some bytes to be received yet
        if((bytes_received =! bytes_to_receive) and (res.tHead.ulExt == HIL_PACKET_SEQ_NONE or res.tHead.ulExt == HIL_PACKET_SEQ_LAST))
            throw cifx::Error{ CIFX_DEV_FUNCTION_FAILED, context, "There are bytes to be received yet, but CIFX device reports end of proceeded" };
    }
}

/* ================================================================================================================================ */

} // End namespace cifx::ethercat
