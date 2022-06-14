/* ============================================================================================================================ *//**
 * @file       profiled_position.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 2nd June 2022 12:53:22 pm
 * @modified   Monday, 13th June 2022 5:41:26 pm
 * @project    engineering-thesis
 * @brief      Definitions of configuration constants & structures related to 'Profiled Position' of the Elmo driver's Objects 
 *             Dictionary (OD)
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __ETHERCAT_DEVICES_ELMO_CONFIG_PROFILED_POSITION_H__
#define __ETHERCAT_DEVICES_ELMO_CONFIG_PROFILED_POSITION_H__

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
enum class MotionProfile : TypeRepresentation<registers::MOTION_PROFILE_TYPE.type.get_id()> {
    Trapezoidal = 0
};

/* ================================================================================================================================ */

} // End namespace ethercat::devices::elmo::config

/* ================================================================================================================================ */

#endif
