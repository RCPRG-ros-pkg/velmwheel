/* ============================================================================================================================ *//**
 * @file       debug.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Monday, 13th June 2022 3:25:21 pm
 * @modified   Friday, 1st July 2022 3:57:06 pm
 * @project    engineering-thesis
 * @brief      Definition of debug-related methods of the Elmo driver class
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __ETHERCAT_DEVICES_ELMO_DEBUG_H__
#define __ETHERCAT_DEVICES_ELMO_DEBUG_H__

/* =========================================================== Includes =========================================================== */

// Private includes
#include "ethercat/devices/elmo.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace ethercat::devices {

/* ================================================= Public debug-related methods ================================================= */

template<typename SlaveImplementationT>
typename Elmo<SlaveImplementationT>::SlaveInterface &
Elmo<SlaveImplementationT>::get_slave() {
    return slave;
}


template<typename SlaveImplementationT>
const typename Elmo<SlaveImplementationT>::SlaveInterface &
Elmo<SlaveImplementationT>::get_slave() const {
    return slave;
}

/* ================================================================================================================================ */

} // End namespace ethercat::devices

#endif
