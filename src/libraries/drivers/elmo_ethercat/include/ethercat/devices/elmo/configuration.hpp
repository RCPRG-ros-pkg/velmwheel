/* ============================================================================================================================ *//**
 * @file       elmo.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Monday, 13th June 2022 3:25:21 pm
 * @modified   Monday, 13th June 2022 8:15:54 pm
 * @project    engineering-thesis
 * @brief      Definition of configruation methods of the Elmo driver class
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __ETHERCAT_DEVICES_ELMO_CONFIGRUATION__H__
#define __ETHERCAT_DEVICES_ELMO_CONFIGRUATION__H__

/* =========================================================== Includes =========================================================== */

// Private includes
#include "ethercat/devices/elmo.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace ethercat::devices {

/* ===================================================== Public setup methods ===================================================== */

template<typename SlaveImplementationT>
template<typename HandlerT>
void Elmo<SlaveImplementationT>::set_input_handler(HandlerT &&handler) {
    slave.register_event_handler(SlaveInterface::Event::InputsUpdate, std::forward<HandlerT>(handler));
}


template<typename SlaveImplementationT>
template<typename HandlerT>
void Elmo<SlaveImplementationT>::set_output_handler(HandlerT &&handler) {
    slave.register_event_handler(SlaveInterface::Event::OutputsUpdate, std::forward<HandlerT>(handler));
}

/* =================================================== Public examining methods =================================================== */

template<typename SlaveImplementationT>
Elmo<SlaveImplementationT>::PdoMappingInfo 
Elmo<SlaveImplementationT>::get_pdo_mapping_info() const {
    return PdoMappingInfo {
        .has_digital_inputs  = digital_inputs_pdo.has_value(),
        .has_position        = position_pdo.has_value(),
        .has_velocity        = velocity_pdo.has_value(),
        .has_torque          = torque_pdo.has_value(),
        .has_current         = current_pdo.has_value(),
        .has_target_position = target_position_pdo.has_value(),
        .has_target_velocity = target_velocity_pdo.has_value(),
        .has_target_torque   = target_torque_pdo.has_value()
    };
}

/* ================================================= Device-configuration methods ================================================= */

template<typename SlaveImplementationT>
elmo::config::ModesOfOperation Elmo<SlaveImplementationT>::read_mode_of_operation() const {
    return mode_of_operation_sdo.upload();
}


template<typename SlaveImplementationT>
void Elmo<SlaveImplementationT>::read_mode_of_operation(elmo::config::ModesOfOperation mode) {
    mode_of_operation_sdo.download(mode);
}

/* ================================================================================================================================ */

} // End namespace ethercat::devices

#endif
