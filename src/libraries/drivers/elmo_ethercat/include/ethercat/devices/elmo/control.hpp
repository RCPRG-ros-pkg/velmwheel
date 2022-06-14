/* ============================================================================================================================ *//**
 * @file       control.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Monday, 13th June 2022 3:25:21 pm
 * @modified   Monday, 13th June 2022 6:23:15 pm
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
    return control_pdo.download(command);
}


template<typename SlaveImplementationT>
Elmo<SlaveImplementationT>::Command Elmo<SlaveImplementationT>::get_command() const {
    return control_pdo.upload();
}


template<typename SlaveImplementationT>
Elmo<SlaveImplementationT>::State
Elmo<SlaveImplementationT>::get_state() const {

    using namespace elmo::config;
    using namespace elmo::config::statusword::State;

    // Read current state
    Statusword status = statusword_pdo.get();

    // Parse current state
    if((status & NotReadyToSwitchOnMask  ) == NotReadyToSwitchOn ) return State::NotReadyToSwitchOn;
    if((status & SwitchedOnDisabledMask  ) == SwitchedOnDisabled ) return State::SwitchedOnDisabled;
    if((status & ReadyToSwitchOnMask     ) == ReadyToSwitchOn    ) return State::ReadyToSwitchOn;
    if((status & SwitchOnMask            ) == SwitchOn           ) return State::SwitchOn;
    if((status & OperationEnabledMask    ) == OperationEnabled   ) return State::OperationEnabled;
    if((status & QuickStopActiveMask     ) == QuickStopActive    ) return State::QuickStopActive;
    if((status & FaultReactionAcitveMask ) == FaultReactionAcitve) return State::FaultReactionAcitve;
    if((status & FaultMask               ) == Fault              ) return State::Fault;

    // If non of states parsed, throw error
    throw std::out_of_range{ 
        "[ethercat::devices::Elmo::get_state] Invalid 'Statusword' has been received"
        "(" + std::to_string(status) + ")"
    };
}


template<typename SlaveImplementationT>
elmo::config::DigitalInputs Elmo<SlaveImplementationT>::get_digital_inputs() const {
    if(digital_inputs.has_value()) 
        return digital_inputs.get();
    else
        throw std::out_of_range{ 
            "[ethercat::devices::Elmo::get_digital_inputs] 'Digital inputs' objects is not mapped into PDi by the slave device" 
        };
}

/* ================================================================================================================================ */

} // End namespace ethercat::devices

#endif
