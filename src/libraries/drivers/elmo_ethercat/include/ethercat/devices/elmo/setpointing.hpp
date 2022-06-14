/* ============================================================================================================================ *//**
 * @file       setpointing.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Monday, 13th June 2022 3:25:21 pm
 * @modified   Monday, 13th June 2022 6:23:58 pm
 * @project    engineering-thesis
 * @brief      Definition of methods of the Elmo driver class setting arget control values for servodriver's control loops
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __ETHERCAT_DEVICES_SETPOINTING_H__
#define __ETHERCAT_DEVICES_SETPOINTING_H__

/* =========================================================== Includes =========================================================== */

// Private includes
#include "ethercat/devices/elmo.hpp"
#include "ethercat/devices/elmo/conversions.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace ethercat::devices {

/* ============================================== Public measurements-related methods ============================================= */

template<typename SlaveImplementationT>
void Elmo<SlaveImplementationT>::set_position(double position) {

    // Check if PDO is mapped into the PDI; if so, set it's value after units conversion
    if(target_position_pdo.has_value()) 
        
        target_position_pdo.set(
            elmo::conversions::motor_encoder_to_gearing_rad(config, position));
            
    // Otherwise, throw error
    else
        throw std::out_of_range { 
            "[ethercat::devices::Elmo::get_position] 'Position actual value' objects is not mapped into PDi by the slave device" };
}


template<typename SlaveImplementationT>
void Elmo<SlaveImplementationT>::set_velocity(double velocity) {

    // Check if PDO is mapped into the PDI; if so, set it's value after units conversion
    if(target_velocity_pdo.has_value()) 
        
        target_velocity_pdo.set(
            elmo::conversions::motor_encoder_to_gearing_rad_vel(config, velocity));
            
    // Otherwise, throw error
    else
        throw std::out_of_range { 
            "[ethercat::devices::Elmo::get_velocity] 'Velocity actual value' objects is not mapped into PDi by the slave device" };
}


template<typename SlaveImplementationT>
void Elmo<SlaveImplementationT>::set_torque(double torque) {

    // Check if PDO is mapped into the PDI; if so, set it's value after units conversion
    if(target_torque_pdo.has_value()) 
        
        target_torque_pdo.set(
            elmo::conversions::motor_torque_to_gearing_torque(config, torque));
            
    // Otherwise, throw error
    else
        throw std::out_of_range { 
            "[ethercat::devices::Elmo::get_torque] 'Torque actual value' objects is not mapped into PDi by the slave device" };
}

/* ================================================================================================================================ */

} // End namespace ethercat::devices

#endif
