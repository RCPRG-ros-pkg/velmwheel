/* ============================================================================================================================ *//**
 * @file       registers.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Tuesday, 31st May 2022 11:56:17 am
 * @modified   Monday, 13th June 2022 11:52:15 am
 * @project    engineering-thesis
 * @brief      Definitions of CoE objects of the EtherCAT driver for proprietary IMU sensor board by B. Kaczor
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __ETHERCAT_DEVICES_IMU_REGISTERS_H__
#define __ETHERCAT_DEVICES_IMU_REGISTERS_H__

/* =========================================================== Includes =========================================================== */

// Standard includes
#include <cstdint>
#include <string_view>
// Private includes
#include "ethercat/descriptors.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace ethercat::devices::imu::registers {

/* ============================================================= Types ============================================================ */

/// Alias for Register descriptor type
using Register = ethercat::descriptors::Object<>;
/// Alias for Type type
using Type = ethercat::descriptors::types::Type<>;

/* ==================================================== PDO-mappable registers ==================================================== */

/// Description of the "Acceleration X" CoE register
constexpr Register ACCELERATION_X{ 0x2065, "Acceleration X", Type::Int };
/// Description of the "Acceleration Y" CoE register
constexpr Register ACCELERATION_Y{ 0x2066, "Acceleration Y", Type::Int };
/// Description of the "Acceleration Z" CoE register
constexpr Register ACCELERATION_Z{ 0x2067, "Acceleration Z", Type::Int };

/// Description of the "Rotation Speed X" CoE register
constexpr Register ROTATION_SPEED_X{ 0x2068, "Rotation Speed X", Type::Int };
/// Description of the "Rotation Speed Y" CoE register
constexpr Register ROTATION_SPEED_Y{ 0x2069, "Rotation Speed Y", Type::Int };
/// Description of the "Rotation Speed Z" CoE register
constexpr Register ROTATION_SPEED_Z{ 0x206a, "Rotation Speed Z", Type::Int };

/* ========================================================= SDO registers ======================================================== */

/// Description of the "Gyro Temperature X" CoE register
constexpr Register GYRO_TEMPERATURE_X{ 0x21fc, "Gyro Temperature X", Type::UnsignedInt };
/// Description of the "Gyro Temperature Y" CoE register
constexpr Register GYRO_TEMPERATURE_Y{ 0x21fd, "Gyro Temperature Y", Type::UnsignedInt };
/// Description of the "Gyro Temperature Z" CoE register
constexpr Register GYRO_TEMPERATURE_Z{ 0x21fe, "Gyro Temperature Z", Type::UnsignedInt };

/// Description of the "Gyro bias offset X" CoE register
constexpr Register GYRO_BIAS_OFFSET_X{ 0x21ff, "Gyro bias offset X", Type::UnsignedInt };
/// Description of the "Gyro bias offset Y" CoE register
constexpr Register GYRO_BIAS_OFFSET_Y{ 0x2200, "Gyro bias offset Y", Type::UnsignedInt };
/// Description of the "Gyro bias offset Z" CoE register
constexpr Register GYRO_BIAS_OFFSET_Z{ 0x2201, "Gyro bias offset Z", Type::UnsignedInt };

/// Description of the "Accelerometer bias offset X" CoE register
constexpr Register ACCELEROMETER_OFFSET_X{ 0x2202, "Accelerometer bias offset X", Type::UnsignedInt };
/// Description of the "Accelerometer bias offset Y" CoE register
constexpr Register ACCELEROMETER_OFFSET_Y{ 0x2203, "Accelerometer bias offset Y", Type::UnsignedInt };
/// Description of the "Accelerometer bias offset Z" CoE register
constexpr Register ACCELEROMETER_OFFSET_Z{ 0x2204, "Accelerometer bias offset Z", Type::UnsignedInt };

/// Description of the "Digital Filter Settings" CoE register
constexpr Register DIGITAL_FILTER_SETTINGS{ 0x2205, "Digital Filter Settings", Type::UnsignedInt };
/// Description of the "Gyro Range Settings" CoE register
constexpr Register GYRO_RANGE_SETTINGS{ 0x2206, "Gyro Range Settings", Type::UnsignedInt };
/// Description of the "Autonull Gyro Bias" CoE register
constexpr Register AUTONUL_GYRO_BIAS{ 0x2207, "Autonull Gyro Bias", Type::Bool };
/// Description of the "Restore Factory Calibration" CoE register
constexpr Register RESTORE_FACTORY_CALIBRATION{ 0x2208, "Restore Factory Calibration", Type::Bool };
/// Description of the "Precision Autonull Bias" CoE register
constexpr Register PRECISION_GYRO_BIAS{ 0x2209, "Precision Autonull Bias", Type::Bool };

/// Description of the "Product ID" CoE register
constexpr Register PRODUCT_ID{ 0x220a, "Product ID", Type::UnsignedInt };
/// Description of the "Serial Number" CoE register
constexpr Register SERIAL_NUMBER{ 0x220b, "Serial Number", Type::UnsignedInt };

/* ================================================================================================================================ */

} // End namespace ethercat::devices::imu::registers

/* ================================================================================================================================ */

#endif
