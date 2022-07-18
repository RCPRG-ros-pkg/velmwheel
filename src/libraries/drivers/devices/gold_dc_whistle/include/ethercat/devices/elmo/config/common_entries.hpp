/* ============================================================================================================================ *//**
 * @file       common_entries.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 2nd June 2022 12:53:22 pm
 * @modified   Friday, 1st July 2022 2:16:58 pm
 * @project    engineering-thesis
 * @brief      Definitions of configuration constants & structures related to 'Common entries' of the Elmo driver's Objects 
 *             Dictionary (OD)
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __ETHERCAT_DEVICES_ELMO_CONFIG_COMMON_ENTRIES_H__
#define __ETHERCAT_DEVICES_ELMO_CONFIG_COMMON_ENTRIES_H__

/* =========================================================== Includes =========================================================== */

// Standard includes
#include <cstdint>
// Private includes
#include "ethercat/utilities/named_bitset.hpp"
#include "ethercat/devices/elmo/registers.hpp"
#include "ethercat/devices/elmo/config/common.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace ethercat::devices::elmo::config {

/* ========================================================== Definitions ========================================================= */

/**
 * @brief Possible options of the "Abort connection option code" object
 */
enum class AbortConnectionOption : TypeRepresentation<registers::ABORT_CONNECTION_OPTION_CODE.type.get_id()> {
    Malfunction    = 1,
    DisableVoltage = 2,
    QuickStop      = 3
};

/**
 * @brief Possible options of the "Motor type" object
 */
enum class MotorType : TypeRepresentation<registers::MOTOR_TYPE.type.get_id()> {
    NonStandard            = 0,
    DC                     = 1,
    MicroStepStepper       = 9,
    SinusoidalPMBrushless  = 10,
    TrapezoidalPMBrushless = 11
};

/**
 * @brief Parser structure for the "Supported drive modes" object
 */
struct SupportedDriveModes : utilities::named_bitset<registers::SUPPORTED_DRIVE_MODES.type.get_bitsize()> {

    /**
     * @brief Bits-indexer type for the "Supported drive modes" object
     */
    enum Mode : std::size_t {
        ProfiledPosition     = 0,
        ProfiledVelocity     = 2,
        ProfiledTorque       = 3,
        Homing               = 5,
        InterpolatedPosition = 6,
    };

    // Forward assignment operator
    using utilities::named_bitset<registers::SUPPORTED_DRIVE_MODES.type.get_bitsize()>::operator=;
    
};

/**
 * @brief Parser structure for the "Digital inputes" object
 */
struct DigitalInputs : public utilities::named_bitset<registers::DIGITAL_INPUTS.type.get_bitsize()> {

    /**
     * @brief Bits-indexer type for the "Digital inputes" object
     */
    enum Input : std::size_t {
        NegativeLimit = 0,
        PositiveLimit = 1,
        Home          = 2,
        Interlock     = 3,
        IN1           = 16,
        IN2           = 17,
        IN3           = 18,
        IN4           = 19,
        IN5           = 20,
        IN6           = 21,
    };
    
    // Forward assignment operator
    using utilities::named_bitset<registers::DIGITAL_INPUTS.type.get_bitsize()>::operator=;
    
};

/* ================================================================================================================================ */

} // End namespace ethercat::devices::elmo::config

/* ================================================================================================================================ */

#endif
