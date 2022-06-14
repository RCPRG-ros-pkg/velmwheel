/* ============================================================================================================================ *//**
 * @file       conversions.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Monday, 13th June 2022 4:09:48 pm
 * @modified   Monday, 13th June 2022 6:23:09 pm
 * @project    engineering-thesis
 * @brief      Definition of auxiliary data-conversions functions
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __ETHERCAT_DEVICES_ELMO_CONVERSIONS_H__
#define __ETHERCAT_DEVICES_ELMO_CONVERSIONS_H__

/* =========================================================== Includes =========================================================== */

// Standard includes
#include <cmath>
// Private includes
#include "ethercat/devices/elmo.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace ethercat::devices::elmo::conversions {
    
/* =========================================================== Constants ========================================================== */

// PI constant
static constexpr double PI = M_PI;

/* ================================================= Position-converison functions ================================================ */

/**
 * @brief Converts position of the motor's shaft given in @p encoder counts into
 *    position of the gearing's shaft in [rad] based on the given mechanical 
 *    @p config of the motor
 * 
 * @param config 
 *    mechanical configuration of the motor ( @a config.encoder_resolution is assummed
 *    to be already multipled by factor of @c 4 resulting from frequency multiplication
 *    of the quadrature pulse)
 * @param encoder 
 *    number of encoder pulses [1]
 * @returns 
 *    position of the gearing's shaft in [rad] corresponding to the given number of @p encoder 
 *    counts of the motor
 * 
 * @note [1] At the moment the driver does not support configuring ratio between number of
 *    motor's encoder pulses and value of "Current position value". By default it is set by
 *    the servodrive to 1/1 and this value is assummed by the function
 *    (Object 0x6093: Position factor)
 */
template<typename SlaveImplementationT>
constexpr double motor_encoder_to_gearing_rad(const typename Elmo<SlaveImplementationT>::Config &config, int32_t encoder) {
    return (2.0 * PI * encoder / config.encoder_resolution) / config.gear_ratio;
}

/**
 * @brief Converts @p position gearing's shaft in given in [rad] into position of the of the 
 *    motor's shaft given in encoder counts based on the given mechanical @p config of the motor
 * 
 * @param config 
 *    mechanical configuration of the motor ( @a config.encoder_resolution is assummed
 *    to be already multipled by factor of @c 4 resulting from frequency multiplication
 *    of the quadrature pulse)
 * @param position 
 *    position of the gearing's shaft in [rad] [1]
 * @returns 
 *    position of the motors's shaft in encoder counts corresponding to the given @p position 
 *    of gearing's shaft in [rad]
 * 
 * @note [1] At the moment the driver does not support configuring ratio between number of
 *    motor's encoder pulses and value of "Current position value". By default it is set by
 *    the servodrive to 1/1 and this value is assummed by the function
 *    (Object 0x6093: Position factor)
 */
template<typename SlaveImplementationT>
constexpr int32_t gearing_rad_to_motor_encoder(const typename Elmo<SlaveImplementationT>::Config &config, double position) {
    return static_cast<int32_t>(position * config.gear_ratio (2.0 * PI / config.encoder_resolution));
}

/* ================================================= Velocity-converison functions ================================================ */

/**
 * @brief Converts velocity of the motor's shaft given in @p encoder counts per second into
 *    velocity of the gearing's shaft in [rad/s] based on the given mechanical 
 *    @p config of the motor
 * 
 * @param config 
 *    mechanical configuration of the motor ( @a config.encoder_resolution is assummed
 *    to be already multipled by factor of @c 4 resulting from frequency multiplication
 *    of the quadrature pulse)
 * @param encoder 
 *    number of encoder pulses [1]
 * @returns 
 *    velocity of the gearing's shaft in [rad/s] corresponding to the given speed of motor's
 *    shaft @p encoder in counts per second
 * 
 * @note [1] At the moment the driver does not support configuring ratio between number of
 *    motor's encoder pulses and value of "Current velocity value". By default it is set by
 *    the servodrive to 1/1 and this value is assummed by the function
 *    (Object 0x6094: Velocity encoder factor)
 */
template<typename SlaveImplementationT>
constexpr double motor_encoder_to_gearing_rad_vel(const typename Elmo<SlaveImplementationT>::Config &config, int32_t encoder) {
    return (2.0 * PI * encoder / config.encoder_resolution) / config.gear_ratio;
}

/**
 * @brief Converts @p velocity gearing's shaft in given in [rad] into velocity of the of the 
 *    motor's shaft given in encoder counts based on the given mechanical @p config of the motor
 * 
 * @param config 
 *    mechanical configuration of the motor ( @a config.encoder_resolution is assummed
 *    to be already multipled by factor of @c 4 resulting from frequency multiplication
 *    of the quadrature pulse)
 * @param velocity 
 *    velocity of the gearing's shaft in [rad/s] [1]
 * @returns 
 *    velocity of the motors's shaft in encoder counts per second corresponding to the given 
 *    @p velocity of gearing's shaft in [rad/s]
 * 
 * @note [1] At the moment the driver does not support configuring ratio between number of
 *    motor's encoder pulses and value of "Actual velocity value". By default it is set by
 *    the servodrive to 1/1 and this value is assummed by the function
 *    (Object 0x6094: Velocity encoder factor)
 */
template<typename SlaveImplementationT>
constexpr int32_t gearing_rad_to_motor_encoder_vel(const typename Elmo<SlaveImplementationT>::Config &config, double velocity) {
    return static_cast<int32_t>(velocity * config.gear_ratio (2.0 * PI / config.encoder_resolution));
}

/* ================================================== Torque-converison functions ================================================= */

/**
 * @brief Converts torque applied by the motot's shaft given in promiles of the motor's
 *    rated torque (default format of Elmo register) into torque applied by the gearing's 
 *    shaft in [Nm] based on the motor's @p confg
 * 
 * @param config 
 *    mechanical configuration of the motor
 * @param reg 
 *    torque value in Elmo-register format
 * @returns 
 *    torque applied by the gearing's shaft in [Nm]
 */
template<typename SlaveImplementationT>
constexpr double motor_torque_to_gearing_torque(const typename Elmo<SlaveImplementationT>::Config &config, int16_t reg) {
    return (reg * config.rated_torque / 1'000);
}

/**
 * @brief Converts torque applied by the gearing's shaft in [Nm] into torque applied 
 *    by the motot's shaft given in promiles of the motor's rated torque (default format of Elmo register) 
 *    based on the motor's @p confg 
 * 
 * @param config 
 *    mechanical configuration of the motor
 * @param torque 
 *    torque applied by the gearing's shaft in [Nm]
 * @returns 
 *    torque value in Elmo-register format
 */
template<typename SlaveImplementationT>
constexpr int16_t gearing_torque_to_motor_torque(const typename Elmo<SlaveImplementationT>::Config &config, double torque) {
    return static_cast<int16_t>((torque / config.rated_torque) * 1'000);
}

/* ================================================== Torque-converison functions ================================================= */

/**
 * @brief Converts current flowing thgrough the motor's winding in promiles of the motor's
 *    rated current (default format of Elmo register) into current in [Nm] based on the 
 *    motor's @p confg
 * 
 * @param config 
 *    mechanical configuration of the motor
 * @param reg 
 *    current value in Elmo-register format
 * @returns 
 *    current in [A]
 */
template<typename SlaveImplementationT>
constexpr double reg_to_motor_current(const typename Elmo<SlaveImplementationT>::Config &config, int16_t reg) {
    return (reg * config.rated_current / 1'000);
}

/**
 * @brief Converts current flowing thgrough the motor's winding in [Nm] into current in
 *    promiles of the motor's rated current (default format of Elmo register) based on the 
 *    motor's @p confg 
 * 
 * @param config 
 *    mechanical configuration of the motor
 * @param reg 
 *    current in [A]
 * @returns 
 *    current value in Elmo-register format
 */
template<typename SlaveImplementationT>
constexpr int16_t motor_current_to_reg(const typename Elmo<SlaveImplementationT>::Config &config, double current) {
    return static_cast<int16_t>((current / config.rated_current) * 1'000);
}

/* ================================================================================================================================ */

} // End namespace ethercat::devices::elmo::conversions

#endif
