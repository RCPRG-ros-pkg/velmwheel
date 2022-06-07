/* ============================================================================================================================ *//**
 * @file       base_gazebo.real.cpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Tuesday, 8th March 2022 5:16:22 pm
 * @modified   Wednesday, 25th May 2022 11:48:56 pm
 * @project    engineering-thesis
 * @brief      Implementation of the WUT Velmwheel robot
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

/* =========================================================== Includes =========================================================== */

#include <cmath>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>
#include "velmwheel/gazebo/base_gazebo.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace gazebo { 

/* ======================================================= Cyclical routines ====================================================== */

void Base::OnLoadImpl(
    gazebo::physics::LinkPtr &wheel_rr,
    gazebo::physics::LinkPtr &wheel_rl,
    gazebo::physics::LinkPtr &wheel_fr,
    gazebo::physics::LinkPtr &wheel_fl
) {

    // Get handles to Gazebo models of the _collision engines of the robot's wheels
    wheel_rr_collision = wheel_rr->GetCollision("wheel_rr_collision");
    wheel_rl_collision = wheel_rl->GetCollision("wheel_rl_collision");
    wheel_fr_collision = wheel_fr->GetCollision("wheel_fr_collision");
    wheel_fl_collision = wheel_fl->GetCollision("wheel_fl_collision");
    // If failed to parse wheel_s, report error and exit
    if(not wheel_rr_collision or not wheel_rl_collision or not wheel_fr_collision or not wheel_fl_collision)
        RCLCPP_FATAL(node->get_logger(),  "[base_gazebo] Failed to parse 'wheel_*_collision' models from the robot descriptio");

    // Log info message
    RCLCPP_INFO(node->get_logger(), "Running 'base_gazebo' plugin (realistic implementation)...");
}


velmwheel_msgs::msg::Encoders Base::OnUpdateImpl(const ignition::math::Pose3d &model_pose) {

    using WheelEnum = velmwheel_msgs::msg::WheelEnum;

    // Get orientation of the robot in the simulation with respct to the world's frame
    const ignition::math::Quaternion model_rot = model_pose.Rot();
    
    /**
     * @note Configure firection of the vector of friction for each wheel. Direction of the vector
     *    with the higher friction coefficient (i.e. parallel to the roller's rotation axis) is set
     *    in the local reference frame of the wheel. As the friction vector has to be expressed
     *    in the local reference frame of the wheel, two actions must be performed on the vector given
     *    in the robots's reference frame:
     * 
     *       1) vector must be rotated according to the robot's orientation in the World
     *          frame reference so that the result is expressed in the World's frame reference
     *       *) vector must be placed in the O point of the wheel's reference frame; such an operation
     *          does not change the vector's shape as it consist of transition and no rotation
     *       2) the vector must be rotated by the current angle of the wheel and in the opposite 
     *          direction to it's rotation so that the resulting vector is expressed in the local
     *          frame reference of the wheel
     */
    auto set_friction_vector = [&model_rot](
        physics::CollisionPtr collision_model,
        double x_component,
        double y_component
    ) {
        collision_model->GetSurface()->FrictionPyramid()->direction1 =
            collision_model->WorldPose().Rot().RotateVectorReverse(
                model_rot.RotateVector(ignition::math::Vector3(x_component, y_component, 0.0))
            );
    };

    /**
     * @brief [original author's note] Set magnitude of the rotation vector's component to sqrt(2)/2 
     *    to form a unary vectors constituting angle of multiple of 45° with the model's 
     *    orientation vector (it's angle of the rollers with respect to wheel's rotation axis)
     * 
     * @note The constexpr expressions can be used thanks to GCC P1383 extension of the
     *    libstdc++ standard
     */
    #ifdef __GNUC__
    constexpr double UNARY_VECTOR_COMPONENT = std::sqrt(2.0) / 2;
    #else
    const double UNARY_VECTOR_COMPONENT = std::sqrt(2.0) / 2;
    #endif

    /**
     * @brief Set friction vectors of each wheel
     * 
     * @note Direction of the mu1 (first component of the friction, <fdir1>) is the same for
     *    fl-rr and fr-rl pairs of wheels as diretion of rollers is the same. That was not the
     *    case in the original model as the original implementation was calculated for wheels'
     *    rollers in the setup described in [1] and [3]. The actual setup in the WUT Velmwheel 
     *    robot is identical with that described in [4]. The implementation has been corrected.
     * 
     * @see [1] https://www.researchgate.net/publication/269368896_Structures_of_the_Omnidirectional_Robots_with_Swedish_Wheels
     * @see [2] https://journals.sagepub.com/doi/abs/10.1177/0954406219843568
     * @see [3] https://core.ac.uk/download/pdf/48633343.pdf
     * @see [4] https://research.ijcaonline.org/volume113/number3/pxc3901586.pdf
     */
    set_friction_vector(wheel_rr_collision,  UNARY_VECTOR_COMPONENT, -UNARY_VECTOR_COMPONENT);
    set_friction_vector(wheel_rl_collision,  UNARY_VECTOR_COMPONENT,  UNARY_VECTOR_COMPONENT);
    set_friction_vector(wheel_fr_collision,  UNARY_VECTOR_COMPONENT,  UNARY_VECTOR_COMPONENT);
    set_friction_vector(wheel_fl_collision,  UNARY_VECTOR_COMPONENT, -UNARY_VECTOR_COMPONENT);
    
    velmwheel_msgs::msg::Encoders encoders;

    // Prepare current encoders readings (angles)
    encoders.encoders[WheelEnum::REAR_LEFT].angle   = motor_rl->Position(0);
    encoders.encoders[WheelEnum::REAR_RIGHT].angle  = motor_rr->Position(0);
    encoders.encoders[WheelEnum::FRONT_LEFT].angle  = motor_fl->Position(0);
    encoders.encoders[WheelEnum::FRONT_RIGHT].angle = motor_fr->Position(0);
    // Prepare current encoders readings (velocities)
    encoders.encoders[WheelEnum::REAR_LEFT].velocity   = motor_rl->GetVelocity(0);
    encoders.encoders[WheelEnum::REAR_RIGHT].velocity  = motor_rr->GetVelocity(0);
    encoders.encoders[WheelEnum::FRONT_LEFT].velocity  = motor_fl->GetVelocity(0);
    encoders.encoders[WheelEnum::FRONT_RIGHT].velocity = motor_fr->GetVelocity(0);

    return encoders;
}

/* ================================================================================================================================ */

} // End namespace gazebo

/* ===================================================== Common implementation ==================================================== */

#include "base_gazebo.common.cpp"

/* ================================================================================================================================ */
