/* ============================================================================================================================ *//**
 * @file       common.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Sunday, 13th March 2022 10:34:29 pm
 * @modified   Friday, 15th July 2022 4:35:12 pm
 * @project    engineering-thesis
 * @brief      Set of common enties related to the WUT Velmwheel robot
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __velmwheel_model_velmwheel_common_h__
#define __velmwheel_model_velmwheel_common_h__

/* =========================================================== Includes =========================================================== */

// Standard includes
#include <string_view>
// Private includes
#include "velmwheel_msgs/msg/wheel_enum.hpp"

/* ========================================================== Namespaces ========================================================== */

/**
 * @brief Components of the WUT Velmwheel development environment
 */
namespace velmwheel {

/* ============================================================= Types ============================================================ */

/**
 * @brief Project-wide enumeration of robot's wheels
 */
enum class Wheel : std::size_t {
    RearLeft   = velmwheel_msgs::msg::WheelEnum::REAR_LEFT,
    RearRight  = velmwheel_msgs::msg::WheelEnum::REAR_RIGHT,
    FrontLeft  = velmwheel_msgs::msg::WheelEnum::FRONT_LEFT,
    FrontRight = velmwheel_msgs::msg::WheelEnum::FRONT_RIGHT
};

/**
 * @brief Converts @p wheel to human-readable string
 */
constexpr inline std::string_view wheel_to_str(Wheel wheel) {
    switch(wheel) {
        case Wheel::RearLeft:   return "RearLeft";
        case Wheel::RearRight:  return "RearRight";
        case Wheel::FrontLeft:  return "FrontLeft";
        case Wheel::FrontRight: return "FrontRight";
        default:
            return "<unknown>";
    }
}

/**
 * @brief Converts @p wheel to human-readable string
 */
constexpr inline std::size_t wheel_id(Wheel wheel) {
    return static_cast<std::size_t>(wheel);
}

/* ================================================================================================================================ */

} // End namespace velmwheel

#endif
