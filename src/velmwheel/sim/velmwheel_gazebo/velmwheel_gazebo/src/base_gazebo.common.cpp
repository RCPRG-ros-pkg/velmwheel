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

#include <cmath>
#include <algorithm>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>
#include "velmwheel/gazebo/base_gazebo.hpp"
#include "node_common/communication.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace gazebo { 

/* ==================================================== Initialization routines =================================================== */

void Base::Load(physics::ModelPtr parent, sdf::ElementPtr sdf) {

    /* ------------------------- Initialize ROS interface ------------------------ */

    // Create ROS node
    node = gazebo_ros::Node::Get(sdf);

    // [Subscriber] Create ROS subscriber waiting for angular velocities setpoints for robot's wheels
    *node_common::communication::make_subscriber_builder(setpoint_velocities_sub)
        .node(*node)
        .name(SETPOINT_VELOCITIES_TOPIC_NAME)
        .qos(TOPIC_QUEUE_SIZE)
        .callback(*this, &Base::SetVelocitySetpoints);

    // [Publisher] Create ROS publisher interface for broadcasting encoders measurements
    *node_common::communication::make_publisher_builder(joint_states_pub)
        .node(*node)
        .name(JOINT_STATES_TOPIC_NAME)
        .qos(TOPIC_QUEUE_SIZE);

    // [Publisher] Create ROS publisher interface for broadcasting encoders measurements
    *node_common::communication::make_publisher_builder(encoders_pub)
        .node(*node)
        .name(ENCODERS_TOPIC_NAME)
        .qos(TOPIC_QUEUE_SIZE);
    
    // Create static TF2 frame broadcaster
    static_tf_broadcaster = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*node);

    /* ------------------------ Initialize Gazebo interface ---------------------- */

    // Keep a handle to the model
    model = parent;

    // Bind 'on-world-upate-begin' Gazebo event with the plugin's implementation
    update_connection = event::Events::ConnectWorldUpdateBegin(std::bind(&Base::OnUpdate, this));

    // Get common prefix of the model's elements in the SDF description
    link_prefix = std::string(model->GetName()).append("::");

    // Get handles to Gazebo models of the robot's wheels
    auto wheel_rr = model->GetLink(link_prefix + "wheel_rr");
    auto wheel_rl = model->GetLink(link_prefix + "wheel_rl");
    auto wheel_fr = model->GetLink(link_prefix + "wheel_fr");
    auto wheel_fl = model->GetLink(link_prefix + "wheel_fl");
    // If failed to parse wheel_s, report error and exit
    if(not wheel_rr or not wheel_rl or not wheel_fr or not wheel_fl) {
        auto msg = "[base_gazebo] Failed to parse 'wheel_*' links from the robot descriptio";
        RCLCPP_FATAL(node->get_logger(), msg);
        throw std::runtime_error{ msg };
    }

    // Get handles to Gazebo models of the morots connected to the robot's wheels
    motor_rr = model->GetJoint(link_prefix + "motor_rr");
    motor_rl = model->GetJoint(link_prefix + "motor_rl");
    motor_fr = model->GetJoint(link_prefix + "motor_fr");
    motor_fl = model->GetJoint(link_prefix + "motor_fl");
    // If failed to parse wheel_s, report error and exit
    if(not motor_rr or not motor_rl or not motor_fr or not motor_fl) {
        auto msg = "[base_gazebo] Failed to parse 'motor_*' joints from the robot descriptio";
        RCLCPP_FATAL(node->get_logger(), msg);
        throw std::runtime_error{ msg };
    }

    // Call impolementation-specific initialization
    OnLoadImpl(
        wheel_rr,
        wheel_rl,
        wheel_fr,
        wheel_fl
    );

    /* ----------------------- Provide static transformations -------------------- */

    geometry_msgs::msg::TransformStamped tf_model_transform_msg;

    // Prepare header of the robot's world-position's transformation message
    tf_model_transform_msg.header.stamp    = node->get_clock()->now();
    tf_model_transform_msg.header.frame_id = velmwheel::params::ROBOT_NAME;
    tf_model_transform_msg.child_frame_id  = "base_link";
    // Fill the body of the robot's world-position's transformation message
    tf_model_transform_msg.transform.translation.x = 0;
    tf_model_transform_msg.transform.translation.y = 0;
    tf_model_transform_msg.transform.translation.z = 0;
    tf_model_transform_msg.transform.rotation.x    = ignition::math::Quaternion<double>::Identity.X();
    tf_model_transform_msg.transform.rotation.y    = ignition::math::Quaternion<double>::Identity.Y();
    tf_model_transform_msg.transform.rotation.z    = ignition::math::Quaternion<double>::Identity.Z();
    tf_model_transform_msg.transform.rotation.w    = ignition::math::Quaternion<double>::Identity.W();
    // Update robot's world-position's transformation
    static_tf_broadcaster->sendTransform(tf_model_transform_msg);

}

/* ======================================================= Cyclical routines ====================================================== */

void Base::OnUpdate() {
    
    using WheelEnum = velmwheel_msgs::msg::WheelEnum;

    /* ------------------- Call implementation-specific routine ------------------ */

    // Get current system time-point
    auto now = node->get_clock()->now();
    // Get pose of the robot in the simulation with respct to the world's frame
    const ignition::math::Pose3d model_pose = model->WorldPose();
    // Get velocities of the robot in the simulation with respct to the world's frame
    const ignition::math::Vector3d model_lin_vel = model->WorldLinearVel();
    const ignition::math::Vector3d model_rot_vel = model->WorldAngularVel();

    // Call the routine
    auto encoders = OnUpdateImpl(model_pose);

    /* ------------------ Update simulation model fo the robot ------------------- */

    // Set setpoint for each wheel
    motor_rl->SetVelocity(0, current_controls_setpoint.values[WheelEnum::REAR_LEFT]);
    motor_rr->SetVelocity(0, current_controls_setpoint.values[WheelEnum::REAR_RIGHT]);
    motor_fl->SetVelocity(0, current_controls_setpoint.values[WheelEnum::FRONT_LEFT]);
    motor_fr->SetVelocity(0, current_controls_setpoint.values[WheelEnum::FRONT_RIGHT]);
    
    /* ---------------- Update ROS topics (simulated measurements) --------------- */
    
    sensor_msgs::msg::JointState joint_states_msg;

    // Resize vectors holding states of joints to the target size
    constexpr unsigned WHEELS_NUM = 4;
    joint_states_msg.name.reserve(WHEELS_NUM);
    joint_states_msg.position.reserve(WHEELS_NUM);
    joint_states_msg.velocity.reserve(WHEELS_NUM);
    joint_states_msg.effort.reserve(WHEELS_NUM);
    // Prepare header of the encoders' measurements message
    joint_states_msg.header.stamp    = now;
    joint_states_msg.header.frame_id = velmwheel::params::ROBOT_NAME;
    // Fill the body of the encoders' measurements for rear left wheel
    joint_states_msg.name.push_back("motor_rl");
    joint_states_msg.position.push_back(encoders.encoders[WheelEnum::REAR_LEFT].angle);
    joint_states_msg.velocity.push_back(encoders.encoders[WheelEnum::REAR_LEFT].velocity);
    joint_states_msg.effort.push_back(motor_rl->GetForce(0));
    // Fill the body of the encoders' measurements for rear right wheel
    joint_states_msg.name.push_back("motor_rr");
    joint_states_msg.position.push_back(encoders.encoders[WheelEnum::REAR_RIGHT].angle);
    joint_states_msg.velocity.push_back(encoders.encoders[WheelEnum::REAR_RIGHT].velocity);
    joint_states_msg.effort.push_back(motor_rr->GetForce(0));
    // Fill the body of the encoders' measurements for front left wheel
    joint_states_msg.name.push_back("motor_fl");
    joint_states_msg.position.push_back(encoders.encoders[WheelEnum::FRONT_LEFT].angle);
    joint_states_msg.velocity.push_back(encoders.encoders[WheelEnum::FRONT_LEFT].velocity);
    joint_states_msg.effort.push_back(motor_fl->GetForce(0));
    // Fill the body of the encoders' measurements for front right wheel
    joint_states_msg.name.push_back("motor_fr");
    joint_states_msg.position.push_back(encoders.encoders[WheelEnum::FRONT_RIGHT].angle);
    joint_states_msg.velocity.push_back(encoders.encoders[WheelEnum::FRONT_RIGHT].velocity);
    joint_states_msg.effort.push_back(motor_fr->GetForce(0));
    // Update encoders' measurements
    joint_states_pub->publish(joint_states_msg);

    velmwheel_msgs::msg::EncodersStamped encoders_msg;

    // Prepare header of the encoders' measurements message
    encoders_msg.header.stamp    = now;
    encoders_msg.header.frame_id = velmwheel::params::ROBOT_NAME;
    // Fill message with measurements
    encoders_msg.encoders[WheelEnum::REAR_LEFT].angle      = encoders.encoders[WheelEnum::REAR_LEFT].angle;
    encoders_msg.encoders[WheelEnum::REAR_RIGHT].angle     = encoders.encoders[WheelEnum::REAR_RIGHT].angle;
    encoders_msg.encoders[WheelEnum::FRONT_LEFT].angle     = encoders.encoders[WheelEnum::FRONT_LEFT].angle;
    encoders_msg.encoders[WheelEnum::FRONT_RIGHT].angle    = encoders.encoders[WheelEnum::FRONT_RIGHT].angle;
    encoders_msg.encoders[WheelEnum::REAR_LEFT].velocity   = encoders.encoders[WheelEnum::REAR_LEFT].velocity;
    encoders_msg.encoders[WheelEnum::REAR_RIGHT].velocity  = encoders.encoders[WheelEnum::REAR_RIGHT].velocity;
    encoders_msg.encoders[WheelEnum::FRONT_LEFT].velocity  = encoders.encoders[WheelEnum::FRONT_LEFT].velocity;
    encoders_msg.encoders[WheelEnum::FRONT_RIGHT].velocity = encoders.encoders[WheelEnum::FRONT_RIGHT].velocity;
    
    // Publish the message
    encoders_pub->publish(encoders_msg);
}


void Base::SetVelocitySetpoints(const velmwheel_msgs::msg::Wheels &msg) {

    // Helper macro setting field of the current_controls_setpoint structure
    #define SET_WHEEL_SPEED(wheel)                                       \
        if(!std::isnan(msg.values[wheel]))                               \
            current_controls_setpoint.values[wheel] = msg.values[wheel];

    // Set wheels' speed septoints
    SET_WHEEL_SPEED(velmwheel_msgs::msg::WheelEnum::REAR_LEFT);
    SET_WHEEL_SPEED(velmwheel_msgs::msg::WheelEnum::REAR_RIGHT);
    SET_WHEEL_SPEED(velmwheel_msgs::msg::WheelEnum::FRONT_LEFT);
    SET_WHEEL_SPEED(velmwheel_msgs::msg::WheelEnum::FRONT_RIGHT);

    // Remove macro
    #undef SET_WHEEL_SPEED
}

/* ================================================================================================================================ */

// Register class as Gazebo plugin
GZ_REGISTER_MODEL_PLUGIN(Base)

/* ================================================================================================================================ */

} // End namespace gazebo
