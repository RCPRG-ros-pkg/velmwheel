/* ============================================================================================================================ *//**
 * @file       master.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 18th May 2022 9:50:06 am
 * @modified   Wednesday, 29th June 2022 8:43:30 pm
 * @project    engineering-thesis
 * @brief      Definition of inline methods and methods templates of the Master class representing master device on the EtherCAT 
 *             bus
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __CIFX_ETHERCAT_MASTER_MASTER_H__
#define __CIFX_ETHERCAT_MASTER_MASTER_H__

/* =========================================================== Includes =========================================================== */

// Private includes
#include "cifx/ethercat/master.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace cifx::ethercat {

/* ===================================================== Public static methods ==================================================== */

constexpr std::string_view Master::state_to_str(ExtendedState state) {
    switch(state) {
        case ExtendedState::Init:            return "Init";
        case ExtendedState::Preop:           return "Preop";
        case ExtendedState::Safeop:          return "Safeop";
        case ExtendedState::Op:              return "Op";
        case ExtendedState::Busoff:          return "Busoff";
        case ExtendedState::LeaveOp:         return "LeaveOp";
        case ExtendedState::Busscan:         return "Busscan";
        case ExtendedState::BusscanComplete: return "BusscanComplete";
        default:
            return "<Unknown>";
    }
}

/* ======================================== Public CIFX-specific methods (comm-area mapping) ====================================== */

constexpr auto Master::CyclicMappingInfo::RxEntry::type_to_str(Type type) {
    switch(type){
        case Type::Unused:           return "Unused";
        case Type::ProcessData:      return "ProcessData";
        case Type::DcSystime:        return "DcSystime";
        case Type::BrdAlstatus:      return "BrdAlstatus";
        case Type::BrdDcSystimeDiff: return "BrdDcSystimeDiff";
        case Type::WCStateBits:      return "WCStateBits";
        case Type::ExtsyncStatus:    return "ExtsyncStatus";
        default:
            return "<Unknown>";
    }
}


constexpr auto Master::CyclicMappingInfo::TxEntry::type_to_str(Type type) {
    switch(type){
        case Type::Unused:           return "Unused";
        case Type::ProcessData:      return "ProcessData";
        default:
            return "<Unknown>";
    }
}

/* ======================================== Protected EtherCAT I/O methods (implementation) ======================================= */

void Master::read_bus_impl(::ethercat::config::types::Span<uint8_t> pdi_buffer, std::chrono::milliseconds timeout) {
    channel.get_process_data(CIFX_PDI_DATA_AREA).read(input_pdi_data_offset, pdi_buffer, timeout);
}


void Master::write_bus_impl(::ethercat::config::types::Span<const uint8_t> pdi_buffer, std::chrono::milliseconds timeout) {
    channel.get_process_data(CIFX_PDI_DATA_AREA).write(output_pdi_data_offset, pdi_buffer, timeout);
}

/* ================================================================================================================================ */

} // End namespace cifx::ethercat

#endif
