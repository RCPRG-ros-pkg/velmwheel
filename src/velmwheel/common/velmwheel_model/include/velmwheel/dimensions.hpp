/* ============================================================================================================================ *//**
 * @file       dimensions.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Sunday, 13th March 2022 10:34:29 pm
 * @modified   Wednesday, 25th May 2022 11:28:33 pm
 * @project    engineering-thesis
 * @brief      C++ constants describing characteristic dimensions of the WUT Velmwheel robot
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __VELMWHEEL_MODEL_VELMWHEEL_DIMENSIONS_H__
#define __VELMWHEEL_MODEL_VELMWHEEL_DIMENSIONS_H__

/* ========================================================== Namespaces ========================================================== */

/**
 * @brief C++ constants describing characteristic dimensions of the WUT Velmwheel robot
 */
namespace velmwheel::dimensions {

/* ========================================================== Definitions ========================================================= */

/// Radius of the wheel in [m]
constexpr double WHEEL_RADIUS_M = 0.1027;
/// Radius of the wheel's inner part in [m]
constexpr double WHEEL_INNER_RADIUS_M = 0.0741;
/// Radius of the wheel's roller in [m]
constexpr double WHEEL_ROLLER_RADIUS_M = WHEEL_RADIUS_M - WHEEL_INNER_RADIUS_M;
/// Robot's with in [m] (from measured between oposite wheels' centres)
constexpr double ROBOT_WIDTH_M = 0.76;
/// Robot's length in [m] (from measured between pair wheels' centres)
constexpr double ROBOT_LENGTH_M = 0.728;

/* ================================================================================================================================ */

} // End namespace velmwheel::dimensions

#endif
