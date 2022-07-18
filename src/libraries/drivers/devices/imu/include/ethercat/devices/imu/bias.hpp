/* ============================================================================================================================ *//**
 * @file       bias.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 28th April 2022 9:24:14 pm
 * @modified   Friday, 1st July 2022 7:08:32 pm
 * @project    engineering-thesis
 * @brief      Definition of measurements-bias-calibration-related methods of the Imu driver class
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __ETHERCAT_DEVICES_IMU_BIAS_H__
#define __ETHERCAT_DEVICES_IMU_BIAS_H__

/* =========================================================== Includes =========================================================== */

// Private includes
#include "ethercat/devices/imu.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace ethercat::devices {

/* ====================================================== Public I/O methods ====================================================== */

template<typename SlaveImplementationT>
typename Imu<SlaveImplementationT>::AxisValues 
Imu<SlaveImplementationT>::read_acceleration_bias_offsets() {

    // Return readings from the device converted to SI units
    return AxisValues {
        .x = imu::conversions::acceleration_calib_to_si(accelerometer_bias_offset_x_sdo.upload()),
        .y = imu::conversions::acceleration_calib_to_si(accelerometer_bias_offset_y_sdo.upload()),
        .z = imu::conversions::acceleration_calib_to_si(accelerometer_bias_offset_z_sdo.upload())
    };
    
}


template<typename SlaveImplementationT>
void 
Imu<SlaveImplementationT>::write_acceleration_bias_offsets(const AxisValues &offsets) {

    // Write target registers
    accelerometer_bias_offset_x_sdo.download(imu::conversions::si_to_acceleration_calib(offsets.x));
    accelerometer_bias_offset_y_sdo.download(imu::conversions::si_to_acceleration_calib(offsets.y));
    accelerometer_bias_offset_z_sdo.download(imu::conversions::si_to_acceleration_calib(offsets.z));
    
}


template<typename SlaveImplementationT>
typename Imu<SlaveImplementationT>::AxisValues 
Imu<SlaveImplementationT>::read_gyro_bias_offsets() {

    // Return readings from the device converted to SI units
    return AxisValues {
        .x = imu::conversions::gyro_calib_to_si<Imu>(gyro_bias_offset_x_sdo.upload()),
        .y = imu::conversions::gyro_calib_to_si<Imu>(gyro_bias_offset_y_sdo.upload()),
        .z = imu::conversions::gyro_calib_to_si<Imu>(gyro_bias_offset_z_sdo.upload())
    };
    
}

template<typename SlaveImplementationT>
void 
Imu<SlaveImplementationT>::write_gyro_bias_offsets(const AxisValues &offsets) {

    // Write target registers
    gyro_bias_offset_x_sdo.download(imu::conversions::si_to_gyro_calib<Imu>(offsets.x));
    gyro_bias_offset_y_sdo.download(imu::conversions::si_to_gyro_calib<Imu>(offsets.y));
    gyro_bias_offset_z_sdo.download(imu::conversions::si_to_gyro_calib<Imu>(offsets.z));
    
}

/* ================================================================================================================================ */

} // End namespace ethercat::devices

#endif
