/* ============================================================================================================================ *//**
 * @file       device_control.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 2nd June 2022 12:53:22 pm
 * @modified   Monday, 13th June 2022 5:41:15 pm
 * @project    engineering-thesis
 * @brief      Definitions of configuration constants & structures related to 'Device control' of the Elmo driver's Objects 
 *             Dictionary (OD)
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __ETHERCAT_DEVICES_ELMO_CONFIG_DEVICE_CONTROL_H__
#define __ETHERCAT_DEVICES_ELMO_CONFIG_DEVICE_CONTROL_H__

/* =========================================================== Includes =========================================================== */

// Private includes
#include "ethercat/devices/elmo/registers.hpp"
// Implementation includes
#include "ethercat/devices/elmo/config/device_control/controlword.hpp"
#include "ethercat/devices/elmo/config/device_control/statusword.hpp"
#include "ethercat/devices/elmo/config/common.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace ethercat::devices::elmo::config {
    
/* ================================================= Halt, Stop and Fault Objects ================================================= */

/**
 * @brief Possible options of the "Quick stop option code" object
 */
enum class QuickStopOptionCode : TypeRepresentation<registers::QUICK_STOP_OPTION_CODE.type.get_id()> {
    DisableDriveFunction                       = 0,
    SlowDownOnSlowDownRampAndThenDisableDrive  = 1,
    SlowDownOnQuickStopRampAndThenDisableDrive = 2,
    SlowDownOnCurrentLimitAndThenDisableDrive  = 3,
    SlowDownOnSlowDownRampAndStayInQuickStop   = 5,
    SlowDownOnCurrentLimitAndStayInQuickStop   = 7,
};


/**
 * @brief Possible options of the "Shutdown option code" object
 */
enum class ShutdownOptionCode : TypeRepresentation<registers::SHUTDOWN_OPTION_CODE.type.get_id()> {
    DisableDriveFunction                      = 0,
    SlowDownOnSlowDownRampAndThenDisableDrive = 1,
};


/**
 * @brief Possible options of the "Disable operation mode" object
 */
enum class DisableOperationMode : TypeRepresentation<registers::DISABLE_OPERATION_MODE.type.get_id()> {
    DisableDriveFunction                      = 0,
    SlowDownOnSlowDownRampAndThenDisableDrive = 1,
};


/**
 * @brief Possible options of the "Halt option code" object
 */
enum class HaltOptionCode : TypeRepresentation<registers::HALT_OPTION_CODE.type.get_id()> {
    DisableDriveFunction    = 0,
    SlowDownOnSlowDownRamp  = 1,
    SlowDownOnQuickStopRamp = 2,
    SlowDownOnCurrentLimit  = 3,
};


/**
 * @brief Possible options of the "Fault reaction code" object
 */
enum class FaultReactionCode : TypeRepresentation<registers::FAULT_REACTION_CODE.type.get_id()> {
    DisableDriveFunction = 0,
};

/* ================================================================================================================================ */

} // End namespace ethercat::devices::elmo::config

/* ================================================================================================================================ */

#endif
