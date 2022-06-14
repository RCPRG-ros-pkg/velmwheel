/* ============================================================================================================================ *//**
 * @file       modes_of_operation.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 2nd June 2022 12:53:22 pm
 * @modified   Monday, 13th June 2022 5:41:24 pm
 * @project    engineering-thesis
 * @brief      Definitions of configuration constants & structures related to 'Modes of Operation' of the Elmo driver's Objects 
 *             Dictionary (OD)
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __ETHERCAT_DEVICES_ELMO_CONFIG_MODES_OF_OPERATION_H__
#define __ETHERCAT_DEVICES_ELMO_CONFIG_MODES_OF_OPERATION_H__

/* =========================================================== Includes =========================================================== */

// Private includes
#include "ethercat/devices/elmo/registers.hpp"
#include "ethercat/devices/elmo/config/common.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace ethercat::devices::elmo::config {

/* ========================================================== Definitions ========================================================= */

/**
 * @brief Possible options of the "Modes of Operation" object
 */
enum class ModesOfOperation : TypeRepresentation<registers::MODES_OF_OPERATION.type.get_id()> {
    NoMode               = -1,
    ProfiledPosition     = 0,
    Velocity             = 2, ///< Not supported
    ProfiledVelocity     = 3,
    ProfiledRoque        = 4,
    Homing               = 6,
    InterpolatedPosition = 7,
};

/* ================================================================================================================================ */

} // End namespace ethercat::devices::elmo::config

/* ================================================================================================================================ */

#endif
