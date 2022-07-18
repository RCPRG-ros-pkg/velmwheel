/* ============================================================================================================================ *//**
 * @file       interpolated_position.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 2nd June 2022 12:53:22 pm
 * @modified   Monday, 13th June 2022 5:41:22 pm
 * @project    engineering-thesis
 * @brief      Definitions of configuration constants & structures related to 'Interpolated Position' of the Elmo driver's Objects 
 *             Dictionary (OD)
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __ETHERCAT_DEVICES_ELMO_CONFIG_INTERPOLATED_POSITION_H__
#define __ETHERCAT_DEVICES_ELMO_CONFIG_INTERPOLATED_POSITION_H__

/* =========================================================== Includes =========================================================== */

// Private includes
#include "ethercat/devices/elmo/registers.hpp"
#include "ethercat/devices/elmo/config/common.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace ethercat::devices::elmo::config {
    
/* ========================================================== Definitions ========================================================= */

/**
 * @brief Possible options of the "Motion profile type" object
 */
enum class InterpolatedSubMode : TypeRepresentation<registers::INTERPOLATION_SUB_MODE_SELECT.type.get_id()> {
    CubicSpline         = -1,
    LinearInterpolation =  0
};

/* ================================================================================================================================ */

} // End namespace ethercat::devices::elmo::config

/* ================================================================================================================================ */

#endif
