/* ============================================================================================================================ *//**
 * @file       conversions.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 28th April 2022 9:24:14 pm
 * @modified   Tuesday, 31st May 2022 1:12:17 pm
 * @project    engineering-thesis
 * @brief      Definitions of units-connversion functions related to the Imu class
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __ETHERCAT_DEVICES_IMU_CONVERSIONS_H__
#define __ETHERCAT_DEVICES_IMU_CONVERSIONS_H__

/* =========================================================== Includes =========================================================== */

// Standard includes
#include <cstdint>
#include <stdexcept>
// Private includes
#include "ethercat/devices/imu.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace ethercat::devices {

/* =========================================================== Constants ========================================================== */

namespace imu::details {

    // Gravity acceleration in [m/s^2]
    static constexpr double GRAVITY_ACCELERATION = 9.80665;
    // PI constant
    static constexpr double PI = M_PI;
    // Conversion of value in [deg] into value in [rad] 
    static constexpr auto deg_to_rad = [](double deg) { return deg * PI / 180.0; };
    // Conversion of value in [rad] into value in [deg] 
    static constexpr auto rad_to_deg = [](double rad) { return rad * 180.0 / PI; };

}

/* =============================================== Acceleration conversions methods =============================================== */

template<typename SlaveImplementationT>
constexpr double Imu<SlaveImplementationT>::acceleration_measurement_to_si(int16_t measurement) {

    /**
     * @see <a href="https://www.analog.com/media/en/technical-documentation/data-sheets/ADIS16362.pdf#page=3">ADIS16362 Datasheet (Table 1.)</a>  
     */
    
    // Convert measurement to double representation
    auto double_rep = static_cast<double>(measurement);
    
    // Convert measurement from [0.333 * 10^(-3) * g] unit to [m/s^2] ( @note 0.333 * 10^(-3) = 1 / 3003.003 )
    return double_rep / 3003.003 * imu::details::GRAVITY_ACCELERATION;
        
}


template<typename SlaveImplementationT>
constexpr int16_t Imu<SlaveImplementationT>::si_to_acceleration_measurement(double measurement) {

    /**
     * @see <a href="https://www.analog.com/media/en/technical-documentation/data-sheets/ADIS16362.pdf#page=3">ADIS16362 Datasheet (Table 1.)</a>  
     */
    
    // Convert measurement from [m/s^2] unit to [0.333 * 10^(-3) * g]
    auto reg_rep = measurement * 3003.003 / imu::details::GRAVITY_ACCELERATION;
    
    // Convert measurement to register representation
    return static_cast<int16_t>(reg_rep);
        
}


template<typename SlaveImplementationT>
constexpr double Imu<SlaveImplementationT>::acceleration_calib_to_si(uint16_t calibration) {

    /**
     * @see <a href="https://www.analog.com/media/en/technical-documentation/data-sheets/ADIS16362.pdf#page=12">ADIS16362 Datasheet (Table 16.)</a>  
     */

    // Convert 12-bit two-completion value into 16-bit two-completion value
    auto bit16_rep = (static_cast<int16_t>(calibration << 4) >> 4);
    
    // Convert measurement to SI representation
    return acceleration_measurement_to_si(bit16_rep);
    
}


template<typename SlaveImplementationT>
constexpr uint16_t Imu<SlaveImplementationT>::si_to_acceleration_calib(double calibration) {

    /**
     * @see <a href="https://www.analog.com/media/en/technical-documentation/data-sheets/ADIS16362.pdf#page=12">ADIS16362 Datasheet (Table 16.)</a>  
     */
    
    // Convert measurement to integer representation
    auto integer_rep = si_to_acceleration_measurement(calibration);

    // Convert 16-bit two-completion value into 12-bit two-completion value
    return (static_cast<uint16_t>(integer_rep) >> 4);
        
        
}

/* =================================================== Gyro conversions methods =================================================== */

template<typename SlaveImplementationT>
constexpr double Imu<SlaveImplementationT>::gyro_measurement_to_si(GyroRange range_config, int16_t measurement) {

    // Convert measurement to double representation
    double double_rep = static_cast<double>(static_cast<int16_t>(measurement));

    /**
     * @see <a href="https://www.analog.com/media/en/technical-documentation/data-sheets/ADIS16362.pdf#page=3">ADIS16362 Datasheet (Table 1.)</a>  
     */

    // Convert measurement from [°/s] unit to [rad/s] depending on the current range configruation
    switch(range_config) {
        case GyroRange::Range75:  imu::details::deg_to_rad(0.0125 * measurement); break;
        case GyroRange::Range150: imu::details::deg_to_rad(0.025  * measurement); break;
        case GyroRange::Range300: imu::details::deg_to_rad(0.05   * measurement); break;
        default: /* Should not happen */
            throw std::runtime_error{ "[velmwheel::imu_conversions::gyro_measurement_to_si] Invalid range config [BUG]" };
    }
        
}


template<typename SlaveImplementationT>
constexpr int16_t Imu<SlaveImplementationT>::si_to_gyro_measurement(GyroRange range_config, double measurement) {

    /**
     * @see <a href="https://www.analog.com/media/en/technical-documentation/data-sheets/ADIS16362.pdf#page=3">ADIS16362 Datasheet (Table 1.)</a>  
     */

    double reg_rep;

    // Convert measurement from [rad/s] unit to [°/s] depending on the current range configruation
    switch(range_config) {
        case GyroRange::Range75:  reg_rep = imu::details::rad_to_deg(measurement) / 0.0125 ; break;
        case GyroRange::Range150: reg_rep = imu::details::rad_to_deg(measurement) / 0.025  ; break;
        case GyroRange::Range300: reg_rep = imu::details::rad_to_deg(measurement) / 0.05   ; break;
        default: /* Should not happen */
            throw std::runtime_error{ "[velmwheel::imu_conversions::si_to_gyro_measurement] Invalid range config [BUG]" };
    }
    
    // Convert measurement to integer representation
    return static_cast<int16_t>(reg_rep);
        
}


template<typename SlaveImplementationT>
constexpr double Imu<SlaveImplementationT>::gyro_calib_to_si(uint16_t calibration) {

    /**
     * @see <a href="https://www.analog.com/media/en/technical-documentation/data-sheets/ADIS16362.pdf#page=12">ADIS16362 Datasheet (Table 15.)</a>  
     */

    // Convert 13-bit two-completion value into 16-bit two-completion value
    auto bit16_rep = (static_cast<int16_t>(calibration << 3) >> 3);
    
    /**
     * @note Scaling factor of *GYRO_OFF register is equal to the scaling factor for gyro
     *    measurements when +/-75 [°/s] range is configrued
     */

    // Convert measurement to SI representation
    return gyro_measurement_to_si(GyroRange::Range75, bit16_rep);
    
}


template<typename SlaveImplementationT>
constexpr uint16_t Imu<SlaveImplementationT>::si_to_gyro_calib(double calibration) {

    /**
     * @see <a href="https://www.analog.com/media/en/technical-documentation/data-sheets/ADIS16362.pdf#page=12">ADIS16362 Datasheet (Table 15.)</a>  
     */
    
    /**
     * @note Scaling factor of *GYRO_OFF register is equal to the scaling factor for gyro
     *    measurements when +/-75 [°/s] range is configrued
     */
    
    // Convert measurement to integer representation
    auto integer_rep = si_to_gyro_measurement(GyroRange::Range75, calibration);

    // Convert 16-bit two-completion value into 13-bit two-completion value
    return (static_cast<uint16_t>(integer_rep) >> 3);
        
        
}

/* ====================================================== Gyro (temperature) ====================================================== */

template<typename SlaveImplementationT>
constexpr double Imu<SlaveImplementationT>::gyro_temperature_to_si(uint16_t temperature) {

    /**
     * @see <a href="https://www.analog.com/media/en/technical-documentation/data-sheets/ADIS16362.pdf#page=10">ADIS16362 Datasheet (Table 8.)</a>  
     * @see <a href="https://www.analog.com/media/en/technical-documentation/data-sheets/ADIS16362.pdf#page=11">ADIS16362 Datasheet (Table 9.)</a>  
     */

    // Convert temperature
    return 25.0 + static_cast<double>(temperature) * 0.136;
}

/* ================================================================================================================================ */

} // End namespace ethercat::devices

#endif
