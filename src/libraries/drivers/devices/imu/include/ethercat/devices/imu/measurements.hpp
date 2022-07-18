/* ============================================================================================================================ *//**
 * @file       measurements.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 28th April 2022 9:24:14 pm
 * @modified   Friday, 1st July 2022 7:05:57 pm
 * @project    engineering-thesis
 * @brief      Definition of measurements-related methods of the Imu driver class
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __ETHERCAT_DEVICES_IMU_MEASUREMENTS_H__
#define __ETHERCAT_DEVICES_IMU_MEASUREMENTS_H__

/* =========================================================== Includes =========================================================== */

// Private includes
#include "ethercat/devices/imu.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace ethercat::devices {

/* ===================================================== Public setup methods ===================================================== */

template<typename SlaveImplementationT>
template<typename HandlerT>
void Imu<SlaveImplementationT>::set_measurement_read_handler(HandlerT &&handler) {
    slave.register_event_handler(SlaveInterface::Event::InputsUpdate, std::forward<HandlerT>(handler));
}

/* ====================================================== Public I/O methods ====================================================== */

template<typename SlaveImplementationT>
typename Imu<SlaveImplementationT>::AxisValues 
Imu<SlaveImplementationT>::get_acceleration_measurements() {
    return AxisValues {
        .x = imu::conversions::acceleration_measurement_to_si(acceleration_x_pdo.get()),
        .y = imu::conversions::acceleration_measurement_to_si(acceleration_y_pdo.get()),
        .z = imu::conversions::acceleration_measurement_to_si(acceleration_z_pdo.get())
    };
}


template<typename SlaveImplementationT>
typename Imu<SlaveImplementationT>::AxisValues 
Imu<SlaveImplementationT>::get_gyro_measurements() {
    return AxisValues {
        .x = imu::conversions::gyro_measurement_to_si<Imu>(gyro_range, rotation_speed_x_pdo.get()),
        .y = imu::conversions::gyro_measurement_to_si<Imu>(gyro_range, rotation_speed_y_pdo.get()),
        .z = imu::conversions::gyro_measurement_to_si<Imu>(gyro_range, rotation_speed_z_pdo.get())
    };
}

/* ================================================================================================================================ */

} // End namespace ethercat::devices

#endif
