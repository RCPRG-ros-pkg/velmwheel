/* ============================================================================================================================ *//**
 * @file       base_gazebo.ideal.cpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Tuesday, 8th March 2022 5:16:22 pm
 * @modified   Wednesday, 25th May 2022 11:47:21 pm
 * @project    engineering-thesis
 * @brief      Implementation of the WUT Velmwheel robot
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

/* =========================================================== Includes =========================================================== */

#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>
#include "velmwheel/gazebo/base_gazebo.hpp"
#include "velmwheel/math.hpp"
#include "velmwheel/dimensions.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace gazebo { 

/* ======================================================= Cyclical routines ====================================================== */

void Base::OnLoadImpl(
    gazebo::physics::LinkPtr &wheel_rr,
    gazebo::physics::LinkPtr &wheel_rl,
    gazebo::physics::LinkPtr &wheel_fr,
    gazebo::physics::LinkPtr &wheel_fl
) {
    // Set all frictions related to wheels to zero (friction model is not used in the idealized implementation)
    wheel_rr->GetCollision("wheel_rr_collision")->GetSurface()->FrictionPyramid()->SetMuPrimary   (0.0);
    wheel_rr->GetCollision("wheel_rr_collision")->GetSurface()->FrictionPyramid()->SetMuSecondary (0.0);
    wheel_rl->GetCollision("wheel_rl_collision")->GetSurface()->FrictionPyramid()->SetMuPrimary   (0.0);
    wheel_rl->GetCollision("wheel_rl_collision")->GetSurface()->FrictionPyramid()->SetMuSecondary (0.0);
    wheel_fr->GetCollision("wheel_fr_collision")->GetSurface()->FrictionPyramid()->SetMuPrimary   (0.0);
    wheel_fr->GetCollision("wheel_fr_collision")->GetSurface()->FrictionPyramid()->SetMuSecondary (0.0);
    wheel_fl->GetCollision("wheel_fl_collision")->GetSurface()->FrictionPyramid()->SetMuPrimary   (0.0);
    wheel_fl->GetCollision("wheel_fl_collision")->GetSurface()->FrictionPyramid()->SetMuSecondary (0.0);

    // Log info message
    RCLCPP_INFO(node->get_logger(), "Running 'base_gazebo' plugin (idealized implementation)...");
}


velmwheel_msgs::msg::Encoders Base::OnUpdateImpl(const ignition::math::Pose3d &model_pose) {

    /**
     * @note Equateions based on @see
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

    // Calculate robot's velocities
    geometry_msgs::msg::Twist robot_velocities = velmwheel::math::wheels_to_twist(current_controls_setpoint);

    // Calculate vector of the linear velocity of the model (with respect to the local reference frame)
    ignition::math::Vector3d current_linear_velocity = ignition::math::Vector3d(
        robot_velocities.linear.x,
        robot_velocities.linear.y,
        robot_velocities.linear.z
    );

    // Calculate vector of the angular velocity of the model (with respect to the local reference frame)
    ignition::math::Vector3d current_angular_velocity = ignition::math::Vector3d(
        robot_velocities.angular.x,
        robot_velocities.angular.y,
        robot_velocities.angular.z
    );

    // Set target linear speed of the model
    model->SetLinearVel(model_pose.Rot().RotateVector(current_linear_velocity));
    // Set target angular speed of the model
    model->SetAngularVel(current_angular_velocity);

    using WheelEnum = velmwheel_msgs::msg::WheelEnum;

    /**
     * @note The @ref SetAngularVel(...) method sets vector of angular velocity for all links
     *    of the model for the next simulation's step. This means that wheel joints' speeds
     *    are set to 0. Unfortunatelly this cannot be overwritten with @ref Join::SetValocity(...).
     *    As a result, in the 'ideal' implementation robot's wheels are not moving although
     *    kinematic of the whole robot works correctly
     */
    
    velmwheel_msgs::msg::Encoders encoders;

    // Prepare current encoders readings (angles)
    encoders.encoders[WheelEnum::REAR_LEFT].angle   = motor_rl->Position(0);
    encoders.encoders[WheelEnum::REAR_RIGHT].angle  = motor_rr->Position(0);
    encoders.encoders[WheelEnum::FRONT_LEFT].angle  = motor_fl->Position(0);
    encoders.encoders[WheelEnum::FRONT_RIGHT].angle = motor_fr->Position(0);
    // Prepare current encoders readings (velocities) 
    encoders.encoders[WheelEnum::REAR_LEFT].velocity   = current_controls_setpoint.values[WheelEnum::REAR_LEFT];
    encoders.encoders[WheelEnum::REAR_RIGHT].velocity  = current_controls_setpoint.values[WheelEnum::REAR_RIGHT];
    encoders.encoders[WheelEnum::FRONT_LEFT].velocity  = current_controls_setpoint.values[WheelEnum::FRONT_LEFT];
    encoders.encoders[WheelEnum::FRONT_RIGHT].velocity = current_controls_setpoint.values[WheelEnum::FRONT_RIGHT];

    return encoders;
}

/* ================================================================================================================================ */

} // End namespace gazebo

/* ===================================================== Common implementation ==================================================== */

#include "base_gazebo.common.cpp"

/* ================================================================================================================================ */
