/* ============================================================================================================================ *//**
 * @file       measurements.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 28th April 2022 9:24:14 pm
 * @modified   Monday, 13th June 2022 6:25:06 pm
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
        .x = acceleration_measurement_to_si(acceleration_x_pdo.get()),
        .y = acceleration_measurement_to_si(acceleration_y_pdo.get()),
        .z = acceleration_measurement_to_si(acceleration_z_pdo.get())
    };
}


template<typename SlaveImplementationT>
typename Imu<SlaveImplementationT>::AxisValues 
Imu<SlaveImplementationT>::get_gyro_measurements() {
    return AxisValues {
        .x = gyro_measurement_to_si(gyro_range, rotation_speed_x_pdo.get()),
        .y = gyro_measurement_to_si(gyro_range, rotation_speed_y_pdo.get()),
        .z = gyro_measurement_to_si(gyro_range, rotation_speed_z_pdo.get())
    };
}

/* ================================================================================================================================ */

} // End namespace ethercat::devices

#endif
