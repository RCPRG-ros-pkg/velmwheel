/* ============================================================================================================================ *//**
 * @file       homing.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Monday, 13th June 2022 12:29:36 pm
 * @modified   Monday, 13th June 2022 5:41:19 pm
 * @project    engineering-thesis
 * @brief      Definitions of configuration constants & structures related to 'Homing' of the Elmo driver's Objects 
 *             Dictionary (OD)
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __ETHERCAT_DEVICES_ELMO_CONFIG_HOMING_H__
#define __ETHERCAT_DEVICES_ELMO_CONFIG_HOMING_H__

/* =========================================================== Includes =========================================================== */

// Private includes
#include "ethercat/devices/elmo/registers.hpp"
#include "ethercat/devices/elmo/config/common.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace ethercat::devices::elmo::config {

/* ========================================================== Definitions ========================================================= */

/**
 * @brief Possible options of the "Homing methodity" object
 */
enum class HomingMethod : TypeRepresentation<registers::HOMING_MODE.type.get_id()> {
    NoHoming  = 0,
    Method1   = 1,
    Method2   = 2,
    Method3   = 3,
    Method4   = 4,
    Method5   = 5,
    Method6   = 6,
    Method7   = 7,
    Method8   = 8,
    Method9   = 9,
    Method10  = 10,
    Method11  = 11,
    Method12  = 12,
    Method13  = 13,
    Method14  = 14,
    Method17  = 17,
    Method18  = 18,
    Method19  = 19,
    Method20  = 20,
    Method21  = 21,
    Method22  = 22,
    Method23  = 23,
    Method24  = 24,
    Method25  = 25,
    Method26  = 26,
    Method27  = 27,
    Method28  = 28,
    Method29  = 29,
    Method30  = 30,
    Method33  = 33,
    Method34  = 34,
    Method35  = 35
};

/* ================================================================================================================================ */

} // End namespace ethercat::devices::elmo::config

/* ================================================================================================================================ */

#endif
