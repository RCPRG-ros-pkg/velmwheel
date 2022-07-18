/* ============================================================================================================================ *//**
 * @file       slave.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 18th May 2022 9:50:06 am
 * @modified   Tuesday, 28th June 2022 4:01:14 pm
 * @project    engineering-thesis
 * @brief      Definition of inline methods and methods templates of the Slave class representing slave device on the EtherCAT bus
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __CIFX_ETHERCAT_SLAVE_SLAVE_H__
#define __CIFX_ETHERCAT_SLAVE_SLAVE_H__

/* =========================================================== Includes =========================================================== */

// Private includes
#include "cifx/ethercat/slave.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace cifx::ethercat {

/* ===================================================== Public static methods ==================================================== */

constexpr std::string_view Slave::state_to_str(ExtendedState state) {
    switch(state) {
        case ExtendedState::Init:      return "Init";
        case ExtendedState::Preop:     return "Preop";
        case ExtendedState::Boot:      return "Boot";
        case ExtendedState::Safeop:    return "Safeop";
        case ExtendedState::Op:        return "Op";
        case ExtendedState::Busoff:    return "Busoff";
        case ExtendedState::InitErr:   return "InitErr";
        case ExtendedState::PreopErr:  return "PreopErr";
        case ExtendedState::BootErr:   return "BootErr";
        case ExtendedState::SafeopErr: return "SafeopErr";
        default:
            return "<Unknown>";
    }
}

/* ==================================================== Protected ctors & dtors =================================================== */

Slave::Slave(
    cifx::Channel &channel,
    ::ethercat::eni::Slave slave_eni,
    std::vector<Pdo<PdoDirection::Input>> &&inputs,
    std::vector<Pdo<PdoDirection::Output>> &&outputs
) : 
    // Construct interface class
    ::ethercat::Slave<Slave> {
        slave_eni,
        std::move(inputs),
        std::move(outputs)
    },
    // Initialize implementation-dependent elements
    channel{ channel }
{ }

/* ================================================================================================================================ */

} // End namespace cifx::ethercat

#endif
