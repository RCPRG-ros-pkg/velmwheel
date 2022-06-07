/* ============================================================================================================================ *//**
 * @file       math.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Sunday, 13th March 2022 10:34:29 pm
 * @modified   Wednesday, 25th May 2022 11:30:12 pm
 * @project    engineering-thesis
 * @brief      Routines providing abstract mathematical models related to the WUT Velmwheel robot
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __VELMWHEEL_MODEL_VELMWHEEL_CONVERSIONS_H__
#define __VELMWHEEL_MODEL_VELMWHEEL_CONVERSIONS_H__

/* =========================================================== Includes =========================================================== */

// System includes
#include <tuple>
// Standard messages includes
#include "geometry_msgs/msg/twist.hpp"
// Private includes
#include "velmwheel_msgs/msg/wheel_enum.hpp"
#include "velmwheel_msgs/msg/wheels.hpp"
#include "velmwheel/dimensions.hpp"

/* ========================================================== Namespaces ========================================================== */

/**
 * @brief Routines providing abstract mathematical models related to the WUT Velmwheel robot
 */
namespace velmwheel::math {

/* ========================================================== Definitions ========================================================= */

/**
 * @brief Converts velocity of the WUT Velmwheel robot into the velocity of it's wheels
 * 
 * @param velocity 
 *    velocity of the robot
 * @returns 
 *    velocities of robot's wheels
 * 
 * @note Calculations based on [4]
 * @note [1] and [3] provide opposite twist of rollers and so equations must be 'reversed'
 * @note In [1] Fig. 8 provides wrong visualisation of the wheels' rollers (there is the same
 *    setup as in Velmwheel, but directions are given for the opposite setup)
 * @note [4] presents setup identical to velmwheel
 * 
 * @see [1] https://www.researchgate.net/publication/269368896_Structures_of_the_Omnidirectional_Robots_with_Swedish_Wheels
 * @see [2] https://journals.sagepub.com/doi/abs/10.1177/0954406219843568
 * @see [3] https://core.ac.uk/download/pdf/48633343.pdf
 * @see [4] https://research.ijcaonline.org/volume113/number3/pxc3901586.pdf
 */
velmwheel_msgs::msg::Wheels twist_to_wheels(const geometry_msgs::msg::Twist &velocity) {
    
    velmwheel_msgs::msg::Wheels wheels_velocities{ rosidl_runtime_cpp::MessageInitialization::SKIP };

    // (At compile time) calculate coefficient relating angular velocity with the contribution to the wheel's speeds
    constexpr double ROTATION_COEFFICIENT = (
        velmwheel::dimensions::ROBOT_LENGTH_M +
        velmwheel::dimensions::ROBOT_WIDTH_M
    ) / 2.0;

    using WheelEnum = velmwheel_msgs::msg::WheelEnum;

    // Calculate rotation's contribution to wheels' speeds
    double rotation_contribution = velocity.angular.z * ROTATION_COEFFICIENT;
    // Calculate velocities for all wheels
    wheels_velocities.values[WheelEnum::FRONT_LEFT]  = (velocity.linear.x - velocity.linear.y - rotation_contribution) / velmwheel::dimensions::WHEEL_RADIUS_M;
    wheels_velocities.values[WheelEnum::FRONT_RIGHT] = (velocity.linear.x + velocity.linear.y + rotation_contribution) / velmwheel::dimensions::WHEEL_RADIUS_M;
    wheels_velocities.values[WheelEnum::REAR_LEFT]   = (velocity.linear.x + velocity.linear.y - rotation_contribution) / velmwheel::dimensions::WHEEL_RADIUS_M;
    wheels_velocities.values[WheelEnum::REAR_RIGHT]  = (velocity.linear.x - velocity.linear.y + rotation_contribution) / velmwheel::dimensions::WHEEL_RADIUS_M;
    
    return wheels_velocities;
}


/**
 * @brief Converts velocities of the WUT Velmwheel robot's wheels into the robot's velocity
 * 
 * @param wheels_velocities 
 *    velocities of robot's wheels
 * @returns 
 *    velocity of the robot
 * 
 * @note Calculations based on [4]
 * @note [1] and [3] provide opposite twist of rollers and so equations must be 'reversed'
 * @note In [1] Fig. 8 provides wrong visualisation of the wheels' rollers (there is the same
 *    setup as in Velmwheel, but directions are given for the opposite setup)
 * @note [4] presents setup identical to velmwheel
 * 
 * @see [1] https://www.researchgate.net/publication/269368896_Structures_of_the_Omnidirectional_Robots_with_Swedish_Wheels
 * @see [2] https://journals.sagepub.com/doi/abs/10.1177/0954406219843568
 * @see [3] https://core.ac.uk/download/pdf/48633343.pdf
 * @see [4] https://research.ijcaonline.org/volume113/number3/pxc3901586.pdf
 */
geometry_msgs::msg::Twist wheels_to_twist(const velmwheel_msgs::msg::Wheels &wheels_velocities) {
    
    geometry_msgs::msg::Twist velocity{ rosidl_runtime_cpp::MessageInitialization::ZERO };

    // (At compile time) calculate quarter of wheel's radius
    constexpr double QUARTER_OF_WHEEL_RADIUS = velmwheel::dimensions::WHEEL_RADIUS_M / 4.0;
    // (At compile time) calculate coefficient relating wheels' speeds with the contribution to the angular velocity of the robot
    constexpr double ROTATION_COEFFICIENT = 2.0 / (
        velmwheel::dimensions::ROBOT_LENGTH_M +
        velmwheel::dimensions::ROBOT_WIDTH_M
    );

    auto &fl = wheels_velocities.values[velmwheel_msgs::msg::WheelEnum::FRONT_LEFT];
    auto &fr = wheels_velocities.values[velmwheel_msgs::msg::WheelEnum::FRONT_RIGHT];
    auto &rl = wheels_velocities.values[velmwheel_msgs::msg::WheelEnum::REAR_LEFT];
    auto &rr = wheels_velocities.values[velmwheel_msgs::msg::WheelEnum::REAR_RIGHT];

    // Calculate robot's velocity
    velocity.linear.x  = ( fl + fr + rl + rr) * QUARTER_OF_WHEEL_RADIUS;
    velocity.linear.y  = (-fl + fr + rl - rr) * QUARTER_OF_WHEEL_RADIUS;
    velocity.angular.z = (-fl + fr - rl + rr) * QUARTER_OF_WHEEL_RADIUS * ROTATION_COEFFICIENT;

    return velocity;
}

/* ================================================================================================================================ */

} // End namespace velmwheel::math

#endif
