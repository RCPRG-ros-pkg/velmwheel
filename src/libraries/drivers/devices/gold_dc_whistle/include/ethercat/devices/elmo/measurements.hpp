/* ============================================================================================================================ *//**
 * @file       control.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Monday, 13th June 2022 3:25:21 pm
 * @modified   Monday, 13th June 2022 6:23:42 pm
 * @project    engineering-thesis
 * @brief      Definition of measurements-related methods of the Elmo driver class
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __ETHERCAT_DEVICES_ELMO_MEASUREMENTS_H__
#define __ETHERCAT_DEVICES_ELMO_MEASUREMENTS_H__

/* =========================================================== Includes =========================================================== */

// Private includes
#include "ethercat/devices/elmo.hpp"
#include "ethercat/devices/elmo/conversions.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace ethercat::devices {

/* ============================================== Public measurements-related methods ============================================= */

template<typename SlaveImplementationT>
double Elmo<SlaveImplementationT>::get_position() const {
    
    // Check if PDO is mapped into the PDI; if so, return it's value after units conversion
    if(position_pdo.has_value()) 

        return elmo::conversions::motor_encoder_to_gearing_rad<SlaveImplementationT>(config, position_pdo->get());
        
    // Otherwise, throw error
    else
        throw std::out_of_range{ 
            "[ethercat::devices::Elmo::get_position] 'Position actual value' objects is not mapped into PDi by the slave device" };
}


template<typename SlaveImplementationT>
double Elmo<SlaveImplementationT>::get_velocity() const {
    
    // Check if PDO is mapped into the PDI; if so, return it's value after units conversion
    if(velocity_pdo.has_value()) 

        return elmo::conversions::motor_encoder_to_gearing_rad_vel<SlaveImplementationT>(config, velocity_pdo->get());
        
    // Otherwise, throw error
    else
        throw std::out_of_range{ 
            "[ethercat::devices::Elmo::get_velocity] 'Velocity actual value' objects is not mapped into PDi by the slave device" };
}


template<typename SlaveImplementationT>
double Elmo<SlaveImplementationT>::get_torque() const {
    
    // Check if PDO is mapped into the PDI; if so, return it's value after units conversion
    if(torque_pdo.has_value()) 

        return elmo::conversions::motor_torque_to_gearing_torque<SlaveImplementationT>(config, torque_pdo->get());
        
    // Otherwise, throw error
    else
        throw std::out_of_range{ 
            "[ethercat::devices::Elmo::get_torque] 'Torque actual value' objects is not mapped into PDi by the slave device" };
}


template<typename SlaveImplementationT>
double Elmo<SlaveImplementationT>::get_current() const {
    
    // Check if PDO is mapped into the PDI; if so, return it's value after units conversion
    if(current_pdo.has_value()) 

        return elmo::conversions::reg_to_motor_current<SlaveImplementationT>(config, current_pdo->get());
        
    // Otherwise, throw error
    else
        throw std::out_of_range{ 
            "[ethercat::devices::Elmo::get_current] 'Current actual value' objects is not mapped into PDi by the slave device" };
}

/* ================================================================================================================================ */

} // End namespace ethercat::devices

#endif
