/* ============================================================================================================================ *//**
 * @file       range.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 28th April 2022 9:24:14 pm
 * @modified   Monday, 13th June 2022 6:25:15 pm
 * @project    engineering-thesis
 * @brief      Definition of gyro-sensors-range-related methods of the Imu driver class
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __ETHERCAT_DEVICES_IMU_RANGE_H__
#define __ETHERCAT_DEVICES_IMU_RANGE_H__

/* =========================================================== Includes =========================================================== */

// Private includes
#include "ethercat/devices/imu.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace ethercat::devices {

/* ======================================================== Public methods ======================================================== */

template<typename SlaveImplementationT>
typename Imu<SlaveImplementationT>::GyroRange
Imu<SlaveImplementationT>::read_gyro_range() {

    // Perofrm I/O
    auto setting = gyro_range_settings_sdo.upload();

    // Verify result
    switch(setting) {
        case GyroRange::Range75:
        case GyroRange::Range150:
        case GyroRange::Range300:
            return setting;
        default:
            throw std::out_of_range{ 
                "[ethercat::devices::Imu::read_gyro_range] Invalid range setting read from the device "
                "(" + std::to_string(static_cast<uint16_t>(setting)) + ")"
            };
    }
}


template<typename SlaveImplementationT>
void
Imu<SlaveImplementationT>::write_gyro_range(GyroRange range) {
    gyro_range_settings_sdo.download(range);
}

/* ================================================================================================================================ */

} // End namespace ethercat::devices

#endif
