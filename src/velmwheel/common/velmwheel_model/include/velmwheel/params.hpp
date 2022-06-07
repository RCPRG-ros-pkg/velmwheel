/* ============================================================================================================================ *//**
 * @file       params.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Sunday, 13th March 2022 10:34:29 pm
 * @modified   Wednesday, 25th May 2022 11:31:34 pm
 * @project    engineering-thesis
 * @brief      Set of various constants related to the WUT Velmwheel robot
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __VELMWHEEL_MODEL_VELMWHEEL_PARAMS_H__
#define __VELMWHEEL_MODEL_VELMWHEEL_PARAMS_H__

/* ========================================================== Namespaces ========================================================== */

/**
 * @brief Components of the WUT Velmwheel development environment
 */
namespace velmwheel {

/**
 * @brief Set of various constants related to the WUT Velmwheel robot
 */
namespace params {

/* ========================================================== Definitions ========================================================= */

/// Name of the robot
constexpr auto ROBOT_NAME = "velmwheel";
/// Number robot wheels
constexpr unsigned WHEEL_NUM = 4;

/* ================================================================================================================================ */

} // End namespace params
} // End namespace velmwheel

#endif
