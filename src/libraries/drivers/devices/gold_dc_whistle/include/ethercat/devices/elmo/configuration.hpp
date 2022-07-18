/* ============================================================================================================================ *//**
 * @file       elmo.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Monday, 13th June 2022 3:25:21 pm
 * @modified   Friday, 1st July 2022 2:44:45 pm
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
typename Elmo<SlaveImplementationT>::PdoMappingInfo 
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
elmo::config::ModesOfOperation Elmo<SlaveImplementationT>::read_mode_of_operation(
    std::chrono::milliseconds timeout
) const {
    return mode_of_operation_sdo.upload(timeout);
}


template<typename SlaveImplementationT>
void Elmo<SlaveImplementationT>::write_mode_of_operation(
    elmo::config::ModesOfOperation mode,
    std::chrono::milliseconds timeout
) {
    mode_of_operation_sdo.download(mode, timeout);
}


template<typename SlaveImplementationT>
elmo::config::PolarityConfig
Elmo<SlaveImplementationT>::read_polarity(
    std::chrono::milliseconds timeout
) {
    using namespace elmo::config;

    PolarityConfig ret;

    // Upload the obejct
    PolarityVector obj { PolarityVector::from_value(polarity_sdo.upload(timeout)) };
    // Parse the object
    ret.position_polarity = (obj[PolarityVector::Type::Position] == 0) ? Polarity::Forward : Polarity::Reversed;
    ret.velocity_polarity = (obj[PolarityVector::Type::Velocity] == 0) ? Polarity::Forward : Polarity::Reversed;

    return ret;
}


template<typename SlaveImplementationT>
void Elmo<SlaveImplementationT>::write_polarity(
    elmo::config::PolarityConfig polarity,
    std::chrono::milliseconds timeout
) {
    using namespace elmo::config;
    
    PolarityVector obj;

    // Parse the configruation
    obj[PolarityVector::Type::Position] = (polarity.position_polarity == Polarity::Forward) ? 0 : 1;
    obj[PolarityVector::Type::Velocity] = (polarity.velocity_polarity == Polarity::Forward) ? 0 : 1;

    // Download the object
    polarity_sdo.download(obj.to_value<uint8_t>(), timeout);
}

/* ================================================================================================================================ */

} // End namespace ethercat::devices

#endif
