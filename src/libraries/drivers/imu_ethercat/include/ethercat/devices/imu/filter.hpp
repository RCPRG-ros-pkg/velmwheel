/* ============================================================================================================================ *//**
 * @file       filter.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 28th April 2022 9:24:14 pm
 * @modified   Monday, 13th June 2022 6:24:31 pm
 * @project    engineering-thesis
 * @brief      Definition of digital-filter-related methods of the Imu driver class
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __ETHERCAT_DEVICES_IMU_FILTER_H__
#define __ETHERCAT_DEVICES_IMU_FILTER_H__

/* =========================================================== Includes =========================================================== */

// Private includes
#include "ethercat/devices/imu.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace ethercat::devices {

/* ======================================================== Public methods ======================================================== */

template<typename SlaveImplementationT>
uint16_t Imu<SlaveImplementationT>::read_digital_filter() {
    return digital_filter_settings_sdo.upload();
}


template<typename SlaveImplementationT>
void Imu<SlaveImplementationT>::write_digital_filter(uint16_t filter) {
    digital_filter_settings_sdo.download(filter);
}

/* ================================================================================================================================ */

} // End namespace ethercat::devices

#endif
