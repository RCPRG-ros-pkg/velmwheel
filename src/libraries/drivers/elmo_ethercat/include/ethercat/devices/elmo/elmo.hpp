/* ============================================================================================================================ *//**
 * @file       elmo.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Monday, 13th June 2022 3:25:21 pm
 * @modified   Monday, 13th June 2022 11:38:57 pm
 * @project    engineering-thesis
 * @brief      Definition of special methods of the Elmo driver class
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __ETHERCAT_DEVICES_ELMO_ELMO_H__
#define __ETHERCAT_DEVICES_ELMO_ELMO_H__

/* =========================================================== Includes =========================================================== */

// Private includes
#include "ethercat/devices/elmo.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace ethercat::devices {

/* ===================================================== Public ctors & dtors ===================================================== */

template<typename SlaveImplementationT>
Elmo<SlaveImplementationT>::Elmo(SlaveInterface &slave, const Config &rconfig) :
    
    // Slave handler
    slave{ slave },
    // Motors's configuration
    config { 
        .encoder_resolution { rconfig.encoder_resolution * QuadratureFormatPulseMultiplicationFactor },
        .gear_ratio         { rconfig.gear_ratio                                                     }
    },

    // SDO interfaces
    mode_of_operation_sdo { slave.template get_sdo<decltype(mode_of_operation_sdo)>(elmo::registers::MODES_OF_OPERATION.index ) },
    // PDO entry references
    statusword_pdo  { slave.template get_pdo_entry<PdoDirection::Input> (elmo::registers::STATUSWORD.name ).template get_reference<decltype(statusword_pdo)>()  },
    controlword_pdo { slave.template get_pdo_entry<PdoDirection::Output>(elmo::registers::CONTROLWORD.name).template get_reference<decltype(controlword_pdo)>() }
    
{ 
    /// Auxiliary lambda initializing an optional PDO
    auto initialize_pdo = [this](auto direction, auto &&reg, auto &&pdo) {
        try {
            pdo = this->slave.template get_pdo_entry<direction>(reg.name).
                              template get_reference<decltype(*pdo)>();
        } catch(...) { }
    };

    // Alis for the input direction
    constexpr auto Input = std::integral_constant<PdoDirection, PdoDirection::Input>{};
    // Alis for the output direction
    constexpr auto Output = std::integral_constant<PdoDirection, PdoDirection::Output>{};

    // Status PDOs [IN]
    initialize_pdo(Input, elmo::registers::DIGITAL_INPUTS, digital_inputs_pdo);
    // Measurements PDOs [IN]
    initialize_pdo(Input, elmo::registers::POSITION_ACTUAL_VALUE, position_pdo);
    initialize_pdo(Input, elmo::registers::VELOCITY_ACTUAL_VALUE, velocity_pdo);
    initialize_pdo(Input, elmo::registers::TORQUE_ACTUAL_VALUE,   torque_pdo);
    initialize_pdo(Input, elmo::registers::CURRENT_ACTUAL_VALUE,  current_pdo);
    // Target PDOs [OUT]
    initialize_pdo(Output, elmo::registers::TARGET_POSITION, target_position_pdo);
    initialize_pdo(Output, elmo::registers::TARGET_VELOCITY, target_velocity_pdo);
    initialize_pdo(Output, elmo::registers::TARGET_TORQUE,   target_torque_pdo);

}


template<typename SlaveImplementationT>
Elmo<SlaveImplementationT>::~Elmo() {

    // At teardown unregister handlers for the device
    slave.unregister_event_handler(SlaveInterface::Event::InputsUpdate);
    slave.unregister_event_handler(SlaveInterface::Event::OutputsUpdate);

}

/* ================================================================================================================================ */

} // End namespace ethercat::devices

#endif
