/* ============================================================================================================================ *//**
 * @file       slave.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 18th May 2022 9:50:06 am
 * @modified   Monday, 13th June 2022 5:36:50 am
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

/* ====================================== Protected EtherCAT common methods (implementations) ===================================== */

Slave::State Slave::get_state_impl(std::chrono::milliseconds timeout) {
    return get_state_info(timeout).current_state;
}

/* ================================================================================================================================ */

} // End namespace cifx::ethercat

#endif
