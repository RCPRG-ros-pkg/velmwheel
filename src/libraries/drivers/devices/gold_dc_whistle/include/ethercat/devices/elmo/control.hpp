/* ============================================================================================================================ *//**
 * @file       control.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Monday, 13th June 2022 3:25:21 pm
 * @modified   Wednesday, 29th June 2022 11:16:13 pm
 * @project    engineering-thesis
 * @brief      Definition of device-control methods of the Elmo driver class
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __ETHERCAT_DEVICES_ELMO_CONTROL_H__
#define __ETHERCAT_DEVICES_ELMO_CONTROL_H__

/* =========================================================== Includes =========================================================== */

// Private includes
#include "ethercat/devices/elmo.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace ethercat::devices {

/* ================================================= Public device-control methods ================================================ */

template<typename SlaveImplementationT>
void Elmo<SlaveImplementationT>::set_command(Command command) {
    return control_word_pdo.set(command);
}


template<typename SlaveImplementationT>
typename Elmo<SlaveImplementationT>::State
Elmo<SlaveImplementationT>::get_state() const {

    using namespace elmo::config;
    using namespace elmo::config::statusword::State;

    // Read current state
    Statusword status = status_word_pdo.get();

    // Parse current state
    if((status & NotReadyToSwitchOnMask  ) == NotReadyToSwitchOn ) return State::NotReadyToSwitchOn;
    if((status & SwitchedOnDisabledMask  ) == SwitchedOnDisabled ) return State::SwitchedOnDisabled;
    if((status & ReadyToSwitchOnMask     ) == ReadyToSwitchOn    ) return State::ReadyToSwitchOn;
    if((status & SwitchOnMask            ) == SwitchOn           ) return State::SwitchOn;
    if((status & OperationEnabledMask    ) == OperationEnabled   ) return State::OperationEnabled;
    if((status & QuickStopActiveMask     ) == QuickStopActive    ) return State::QuickStopActive;
    if((status & FaultReactionAcitveMask ) == FaultReactionAcitve) return State::FaultReactionAcitve;
    if((status & FaultMask               ) == Fault              ) return State::Fault;

    std::stringstream ss;

    // Create error message
    ss << "[ethercat::devices::Elmo::get_state] Invalid 'Statusword' has been received "
       << "(0x" << std::hex << std::setfill('0') << std::setw(4) << status.to_value() << ")";

    // If non of states parsed, throw error
    throw std::out_of_range{ ss.str() };
}


template<typename SlaveImplementationT>
elmo::config::DigitalInputs
Elmo<SlaveImplementationT>::get_digital_inputs() const {
    if(digital_inputs_pdo.has_value()) 
        return elmo::config::DigitalInputs{ digital_inputs_pdo->get() };
    else
        throw std::out_of_range{ 
            "[ethercat::devices::Elmo::get_digital_inputs] 'Digital inputs' objects is not mapped into PDi by the slave device" 
        };
}

/* ================================================================================================================================ */

} // End namespace ethercat::devices

#endif
