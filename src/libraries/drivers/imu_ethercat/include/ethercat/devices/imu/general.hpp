/* ============================================================================================================================ *//**
 * @file       bias.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 28th April 2022 9:24:14 pm
 * @modified   Monday, 13th June 2022 6:24:44 pm
 * @project    engineering-thesis
 * @brief      Definition of general configuration methods of the Imu driver class
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __ETHERCAT_DEVICES_IMU_GENERAL_H__
#define __ETHERCAT_DEVICES_IMU_GENERAL_H__

/* =========================================================== Includes =========================================================== */

// Private includes
#include "ethercat/devices/imu.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace ethercat::devices {

/* ======================================================== Public methods ======================================================== */

template<typename SlaveImplementationT>
void
Imu<SlaveImplementationT>::calibrate_gyro_bias() {
    autonul_gyro_bias_sdo.download(true);
}


template<typename SlaveImplementationT>
void
Imu<SlaveImplementationT>::calibrate_precise_gyro_bias() {
    precision_gyro_bias_sdo.download(true);
}


template<typename SlaveImplementationT>
typename Imu<SlaveImplementationT>::AxisValues
Imu<SlaveImplementationT>::get_gyro_temperatures() {

    // Return readings from the device converted to SI units
    return AxisValues {
        .x = gyro_temperature_to_si(gyro_temperature_x_sdo.upload()),
        .y = gyro_temperature_to_si(gyro_temperature_y_sdo.upload()),
        .z = gyro_temperature_to_si(gyro_temperature_z_sdo.upload())
    };

}


template<typename SlaveImplementationT>
typename Imu<SlaveImplementationT>::ProductInfo
Imu<SlaveImplementationT>::get_product_info() {
    return ProductInfo {
        .product_id = product_id_sdo.upload(),
        .serial_num = serial_number_sdo.upload()
    };
}


template<typename SlaveImplementationT>
void
Imu<SlaveImplementationT>::restore_factory_calibration() {
    restore_factory_calibration_sdo.download(true);
}

/* ================================================================================================================================ */

} // End namespace ethercat::devices

#endif
