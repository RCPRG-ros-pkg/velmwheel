/* ============================================================================================================================ *//**
 * @file       registers.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Tuesday, 31st May 2022 11:56:17 am
 * @modified   Monday, 13th June 2022 5:39:25 pm
 * @project    engineering-thesis
 * @brief      Definitions of relevant CoE objects of the EtherCAT driver for Elmo Gold DC Whistle servomotor driver
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __ETHERCAT_DEVICES_ELMO_REGISTERS_H__
#define __ETHERCAT_DEVICES_ELMO_REGISTERS_H__

/* =========================================================== Includes =========================================================== */

// Standard includes
#include <cstdint>
#include <string_view>
// Private includes
#include "ethercat.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace ethercat::devices::elmo::registers {

/* ============================================================= Types ============================================================ */
/// Alias for Register descriptor type
using Register = ethercat::descriptors::Object<>;
/// Alias for Type type
using Type = ethercat::descriptors::types::Type<>;

/* ===================================================== Predefined registers ===================================================== */

/// Description of the "Device Type" CoE register
constexpr Register DEVICE_TYPE{ 0x1000, "Device Type", Type::UnsignedDoubleInt };
/// Description of the "Error register" CoE register
constexpr Register ERROR_REGISTER{ 0x1001, "Error register", Type::UnsignedShortInt };

/* ======================================================== Common entries ======================================================== */

/// Description of the "Abort connection option code" CoE register
constexpr Register ABORT_CONNECTION_OPTION_CODE{ 0x6007, "Abort connection option code", Type::Int };
/// Description of the "Error code" CoE register
constexpr Register ERROR_CODE{ 0x603F, "Error code", Type::UnsignedInt };

/// Description of the "Motor type" CoE register
constexpr Register MOTOR_TYPE{ 0x6402, "Motor type", Type::UnsignedDoubleInt };

/// Description of the "Supported drive modes" CoE register
constexpr Register SUPPORTED_DRIVE_MODES{ 0x6502, "Supported drive modes", Type::UnsignedDoubleInt };
/// Description of the "Digital inputs" CoE register
constexpr Register DIGITAL_INPUTS{ 0x60FD, "Digital Inputs", Type::UnsignedDoubleInt };

/* ======================================================== Device control ======================================================== */

/// Description of the "Controlword" CoE register
constexpr Register CONTROLWORD{ 0x6040, "Control word", Type::UnsignedInt };
/// Description of the "Statusword" CoE register
constexpr Register STATUSWORD{ 0x6541, "Status word", Type::UnsignedInt };

/// Description of the "Quick stop option code" CoE register
constexpr Register QUICK_STOP_OPTION_CODE { 0x605A, "Quick stop option code", Type::Int };
/// Description of the "Shutdown option code" CoE register
constexpr Register SHUTDOWN_OPTION_CODE { 0x605B, "Shutdown option code", Type::Int };
/// Description of the "Disable operation mode" CoE register
constexpr Register DISABLE_OPERATION_MODE { 0x605C, "Disable operation mode", Type::Int };
/// Description of the "Halt option code" CoE register
constexpr Register HALT_OPTION_CODE { 0x605D, "Halt option code", Type::Int };
/// Description of the "Fault reaction code" CoE register
constexpr Register FAULT_REACTION_CODE { 0x605E, "Fault reaction code", Type::Int };

/* ====================================================== Modes of operation ====================================================== */

/// Description of the "Modes of operation" CoE register
constexpr Register MODES_OF_OPERATION { 0x6060, "Modes of operation", Type::ShortInt };
/// Description of the "Modes of operation display " CoE register
constexpr Register MODES_OF_OPERATION_DISPLAY { 0x6061, "Modes of operation display", Type::ShortInt };

/* ============================================================ Factors =========================================================== */

/// Description of the "Polarity" CoE register
constexpr Register POLARITY { 0x607E, "Polarity", Type::UnsignedShortInt };
/// Description of the "Position encoder resolution" CoE register
constexpr Register POSITION_ENCODER_RESOLUTION { 0x608F, "Position encoder resolution", Type::template UnsignedDoubleIntArray<2> };
/// Description of the "Velocity encoder resolution" CoE register
constexpr Register VELOCITY_ENCODER_RESOLUTION { 0x6090, "Velocity encoder resolution", Type::template UnsignedDoubleIntArray<2> };
/// Description of the "Position factor" CoE register
constexpr Register POSITION_FACTOR { 0x6093, "Position factor", Type::template UnsignedDoubleIntArray<2> };
/// Description of the "Velocity encoder factor" CoE register
constexpr Register VELOCITY_ENCODER_FACTOR { 0x6094, "Velocity encoder factor", Type::template UnsignedDoubleIntArray<2> };
/// Description of the "Velocity factor 1" CoE register
constexpr Register VELOCITY_FACTOR_1 { 0x6095, "Velocity factor 1", Type::template UnsignedDoubleIntArray<2> };
/// Description of the "Velocity factor 2" CoE register
constexpr Register VELOCITY_FACTOR_2 { 0x6096, "Velocity factor 2", Type::template UnsignedDoubleIntArray<2> };
/// Description of the "Acceleration factor" CoE register
constexpr Register ACCELERATION_FACTOR { 0x6097, "Acceleration factor", Type::template UnsignedDoubleIntArray<2> };

/* ============================================================ Homing ============================================================ */

/// Description of the "Home offset" CoE register
constexpr Register HOME_OFFSET { 0x607C, "Home offset", Type::DoubleInt };
/// Description of the "Homing mode" CoE register
constexpr Register HOMING_MODE { 0x6098, "Homing mode", Type::ShortInt };
/// Description of the "Homing speeds" CoE register
constexpr Register HOMING_SPEEDS { 0x6099, "Homing speeds", Type::template UnsignedIntArray<2> };
/// Description of the "Homing acceleration" CoE register
constexpr Register HOMING_ACCELERATION { 0x609A, "Homing acceleration", Type::UnsignedInt };

/* ======================================================= Position control ======================================================= */

/// Description of the "Position demand value in position units" CoE register               
constexpr Register POSITION_DEMAND_VALUE_IN_POSITION_UNITS { 0x6062, "Position demand value in position units", Type::DoubleInt };
/// Description of the "Position actual value in increments" CoE register               
constexpr Register POSITION_ACTUAL_VALUE_IN_INCREMENTS { 0x6063, "Position actual value in increments", Type::DoubleInt };
/// Description of the "Position actual value" CoE register               
constexpr Register POSITION_ACTUAL_VALUE { 0x6064, "Position actual value", Type::DoubleInt };
/// Description of the "Following error window" CoE register               
constexpr Register FOLLOWING_ERROR_WINDOW { 0x6065, "Following error window", Type::UnsignedDoubleInt };
/// Description of the "Position window" CoE register               
constexpr Register POSITION_WINDOW { 0x6067, "Position window", Type::UnsignedDoubleInt };
/// Description of the "Position window time out" CoE register
constexpr Register POSITION_WINDOW_TIME_OUT { 0x6068, "Position window time out", Type::UnsignedInt };
/// Description of the "Following error actual value" CoE register               
constexpr Register FOLLOWING_ERROR_ACTUAL_VALUE { 0x60F4, "Following error actual value", Type::Bool /* Lacking documentation */ };
/// Description of the "Position control effort" CoE register               
constexpr Register POSITION_CONTROL_EFFORT { 0x60FA, "Position control effort", Type::Bool /* Lacking documentation */ };
/// Description of the "Position demand value in increments" CoE register               
constexpr Register POSITION_DEMAND_VALUE_IN_INCREMENTS { 0x60FC, "Position demand value in increments", Type::Bool /* Lacking documentation */ };

/* ======================================================= Profiled Position ====================================================== */

/// Description of the "Target position" Coe register
constexpr Register TARGET_POSITION { 0x607A, "Target Position", Type::DoubleInt };
/// Description of the "Position range limit" Coe register
constexpr Register POSITION_RANGE_LIMIT { 0x607B, "Position range limit", Type::template DoubleIntArray<2> };
/// Description of the "Software position limit" Coe register
constexpr Register SOFTWARE_POSITION_LIMIT { 0x607D, "Software position limit", Type::template DoubleIntArray<2> };
/// Description of the "Maximum profile velocity" Coe register
constexpr Register MAXIMUM_PROFILE_VELOCITY { 0x607F, "Maximum profile velocity", Type::UnsignedDoubleInt };
/// Description of the "Profiled velocity" Coe register
constexpr Register PROFILED_VELOCITY { 0x6081, "Profiled velocity", Type::UnsignedDoubleInt };
/// Description of the "End velocity" Coe register
constexpr Register END_VELOCITY { 0x6082, "End velocity", Type::UnsignedDoubleInt };
/// Description of the "Profiled acceleration" Coe register
constexpr Register PROFILED_ACCELERATION { 0x6083, "Profiled acceleration", Type::UnsignedDoubleInt };
/// Description of the "Profiled deceleration" Coe register
constexpr Register PROFILED_DECELERATION { 0x6084, "Profiled deceleration", Type::UnsignedDoubleInt };
/// Description of the "Quick stop deceleration" Coe register
constexpr Register QUICK_STOP_DECELERATION { 0x6085, "Quick stop deceleration", Type::UnsignedDoubleInt };
/// Description of the "Motion profile type" Coe register
constexpr Register MOTION_PROFILE_TYPE { 0x6086, "Motion profile type", Type::Int };
/// Description of the "Maximum acceleration" Coe register
constexpr Register MAXIMUM_ACCELERATION { 0x60C5, "Maximum acceleration", Type::Bool /* Lacking documentation */ };
/// Description of the "Maximum deceleration" Coe register
constexpr Register MAXIMUM_DECELERATION { 0x60C6, "Maximum deceleration", Type::Bool /* Lacking documentation */ };

/* ===================================================== Interpolated Position ==================================================== */

/// Description of the "Interpolation sub mode select" CoE register
constexpr Register INTERPOLATION_SUB_MODE_SELECT { 0x60C0, "Interpolation sub mode select", Type::Int };
/// Description of the "Interpolation data record" CoE register
constexpr Register INTERPOLATION_DATA_RECORD { 0x60C1, "Interpolation data record", Type::DoubleInt /* Custom type required */ };
/// Description of the "Interpolation time period" CoE register
constexpr Register INTERPOLATION_TIME_PERIOD { 0x60C2, "Interpolation time period", Type::template ShortIntArray<2> /* Custom type required */ };
/// Description of the "Interpolation sync definition" CoE register
constexpr Register INTERPOLATION_SYNC_DEFINITION { 0x60C3, "Interpolation sync definition", Type::template UnsignedShortIntArray<2> };
/// Description of the "Interpolation data configuration " CoE register
constexpr Register INTERPOLATION_DATA_CONFIGURATION { 0x60C4, "Interpolation data configuration ", Type::Bool /* Custom type required */ };

/* ======================================================= Profile velocity ======================================================= */

/// Description of the "Velocity sensor actual value" CoE register
constexpr Register VELOCITY_SENSOR_ACTUAL_VALUE { 0x6069, "Velocity sensor actual value", Type::DoubleInt };
/// Description of the "Sensor selection code" CoE register
constexpr Register SENSOR_SELECTION_CODE { 0x606A, "Sensor selection code", Type::Int };
/// Description of the "Velocity demand value" CoE register
constexpr Register VELOCITY_DEMAND_VALUE { 0x606B, "Velocity demand value", Type::DoubleInt };
/// Description of the "Velocity actual value" CoE register
constexpr Register VELOCITY_ACTUAL_VALUE { 0x606C, "Velocity actual value", Type::DoubleInt };
/// Description of the "Velocity window" CoE register
constexpr Register VELOCITY_WINDOW { 0x606D, "Velocity window", Type::UnsignedInt };
/// Description of the "Velocity window time" CoE register
constexpr Register VELOCITY_WINDOW_TIME { 0x606E, "Velocity window time", Type::UnsignedInt };
/// Description of the "Velocity threshold" CoE register
constexpr Register VELOCITY_THRESHOLD { 0x606F, "Velocity threshold", Type::UnsignedInt };
/// Description of the "Velocity threshold time" CoE register
constexpr Register VELOCITY_THRESHOLD_TIME { 0x6070, "Velocity threshold time", Type::UnsignedInt };
/// Description of the "Target velocity" CoE register
constexpr Register TARGET_VELOCITY { 0x60FF, "Target Velocity", Type::DoubleInt };

/* ======================================================== Profiled Torque ======================================================= */

/// Description of the "Target torque" CoE register
constexpr Register TARGET_TORQUE { 0x6071, "Target Torque", Type::Int };
/// Description of the "Max torque" CoE register
constexpr Register MAX_TORQUE { 0x6072, "Max torque", Type::Int };
/// Description of the "Max current" CoE register
constexpr Register MAX_CURRENT { 0x6073, "Max current", Type::Int };
/// Description of the "Torque demand value" CoE register
constexpr Register TORQUE_DEMAND_VALUE { 0x6074, "Torque demand value", Type::Int };
/// Description of the "Motor rated current" CoE register
constexpr Register MOTOR_RATED_CURRENT { 0x6075, "Motor rated current", Type::UnsignedDoubleInt };
/// Description of the "Motor rated torque" CoE register
constexpr Register MOTOR_RATED_TORQUE { 0x6076, "Motor rated torque", Type::UnsignedDoubleInt };
/// Description of the "Torque actual value" CoE register
constexpr Register TORQUE_ACTUAL_VALUE { 0x6077, "Torque actual value", Type::Int };
/// Description of the "Current actual value" CoE register
constexpr Register CURRENT_ACTUAL_VALUE { 0x6078, "Current actual value", Type::Int };
/// Description of the "Torque slope" CoE register
constexpr Register TORQUE_SLOPE { 0x6087, "Torque slope", Type::UnsignedDoubleInt };
/// Description of the "Torque profile type" CoE register
constexpr Register TORQUE_PROFILE_TYPE { 0x6088, "Torque profile type", Type::Int };


/* ================================================================================================================================ */

} // End namespace ethercat::devices::elmo::registers

/* ================================================================================================================================ */

#endif
