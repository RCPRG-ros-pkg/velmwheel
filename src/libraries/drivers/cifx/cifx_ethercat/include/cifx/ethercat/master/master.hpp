/* ============================================================================================================================ *//**
 * @file       master.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 18th May 2022 9:50:06 am
 * @modified   Monday, 13th June 2022 5:38:37 am
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
    
{ }

/* ====================================== Protected EtherCAT common methods (implementations) ===================================== */

Master::State Master::get_state_impl(std::chrono::milliseconds timeout) {

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

/* ======================================== Protected EtherCAT I/O methods (implementation) ======================================= */

void Master::read_bus_impl(ranges::span<uint8_t> pdi_buffer, std::chrono::milliseconds timeout) {
    channel.get_process_data(CIFX_PDI_DATA_AREA).read(PDI_DATA_OFFSET, pdi_buffer, timeout);
}


void Master::write_bus_impl(ranges::span<uint8_t> pdi_buffer, std::chrono::milliseconds timeout) {
    channel.get_process_data(CIFX_PDI_DATA_AREA).write(PDI_DATA_OFFSET, pdi_buffer, timeout);
}

/* ================================================================================================================================ */

} // End namespace cifx::ethercat

#endif
