/* ============================================================================================================================ *//**
 * @file       base_driver.cpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 28th April 2022 9:24:14 pm
 * @modified   Friday, 15th July 2022 3:10:53 pm
 * @project    engineering-thesis
 * @brief      Definition of the driver plugin class for the servodriver EtherCAT slaves of the WUT Velmwheel robot
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

/* =========================================================== Includes =========================================================== */

// External includes
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
// Private includes
#include "velmwheel/base_driver.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace velmwheel {

/* ======================================================= Using namespaces ======================================================= */

using namespace node_common::parameters;
using namespace node_common::communication;

/* ================================================== Private API implementation ================================================== */

std::vector<std::string> BaseDriver::initialize(rclcpp::Node &node) {
    
    // Keep handle to the ROS node
    this->node = &node;

    /* ----------------------------- Initialize parameters --------------------------- */

    auto rear_left_wheel_name   = declare_parameter_and_get(*(this->node), PARAM_NAMESPACE, REAR_LEFT_WHEEL_NAME_PARAM_DESCRIPTOR  );
    auto rear_right_wheel_name  = declare_parameter_and_get(*(this->node), PARAM_NAMESPACE, REAR_RIGHT_WHEEL_NAME_PARAM_DESCRIPTOR );
    auto front_left_wheel_name  = declare_parameter_and_get(*(this->node), PARAM_NAMESPACE, FRONT_LEFT_WHEEL_NAME_PARAM_DESCRIPTOR );
    auto front_right_wheel_name = declare_parameter_and_get(*(this->node), PARAM_NAMESPACE, FRONT_RIGHT_WHEEL_NAME_PARAM_DESCRIPTOR);

    /* ------------------------------- Verify parameters ---------------------------- */

    // Verify whether non-emptydevice names has been given
    if(rear_left_wheel_name->size() == 0)
        rclcpp::exceptions::InvalidParametersException("'eni_wheel_names.rear_left' cannot be an empty string");
    if(rear_right_wheel_name->size() == 0)
        rclcpp::exceptions::InvalidParametersException("'eni_wheel_names.rear_right' cannot be an empty string");
    if(front_left_wheel_name->size() == 0)
        rclcpp::exceptions::InvalidParametersException("'eni_wheel_names.front_left' cannot be an empty string");
    if(front_right_wheel_name->size() == 0)
        rclcpp::exceptions::InvalidParametersException("'eni_wheel_names.front_right' cannot be an empty string");

    /* -------------------------------- Parse parameters ---------------------------- */

    std::vector<std::string> ret(4);

    // Fill vector of slave's names
    ret[Wheels::RearLeft  ] = *rear_left_wheel_name;
    ret[Wheels::RearRight ] = *rear_right_wheel_name;
    ret[Wheels::FrontLeft ] = *front_left_wheel_name;
    ret[Wheels::FrontRight] = *front_right_wheel_name;

    // Return device name
    return ret;

}

void BaseDriver::configure(std::vector<cifx::ethercat::Slave*> slaves) {

    // Check if valid slave interfaces has been given
    if(slaves.size() != Wheels::Num)
        throw std::runtime_error{ "[velmwhee::base_driver] Driver has been configrued with invalid list of slave devices" };

    drivers.reserve(Wheels::Num);
    
    /**
     * @brief Prepare configruation of the servodrivers
     * @details This is current configuration of the robots' wheels
     */
    Driver::Config config {
        .encoder_resolution { 2'500 }, ///< Servomotor with 2500pprev resolution
        .gear_ratio { 50 },            ///< 50-times reduction ration
        .motor_rate_torque{ 1.27 },    /// 1.27Nm rated torque of motors
        .motor_rate_current{ 11 }      /// 11A rated current of motors
    };

    // Construct driver's implementation
    for(std::size_t i = 0; i < Wheels::Num; ++i)
        drivers.emplace_back(*slaves[i], config);

    /* ------------------------- Verify devices' configuration ----------------------- */

    // Verify that all slaves has required PDOs mapped into the PDi
    for(std::size_t i = 0; i < drivers.size(); ++i) {

        auto error = [&i](auto &&msg) {
            std::stringstream ss; ss
                << "[velmwhee::base_driver] " 
                << msg 
                << "(" << velmwheel::wheel_to_str(static_cast<velmwheel::Wheel>(i)) << ")";
            throw std::runtime_error{ ss.str() };
        };

        // Get informations about driver's PDOs
        auto mapping_info = drivers[i].get_pdo_mapping_info();

        // Check if position measurement PDO is mapped
        if(not mapping_info.has_position)
            error("Servodriver has not position-readings PDO mapped into the Process Data Image");
        // Check if velocity measurement PDO is mapped
        if(not mapping_info.has_velocity)
            error("Servodriver has not velocity-readings PDO mapped into the Process Data Image");
        // Check if velocity target PDO is mapped
        if(not mapping_info.has_target_velocity)
            error("Servodriver has not velocity-target PDO mapped into the Process Data Image");

        // Set initial speed of the servo to @c 0 [rad/s]
        drivers[i].set_velocity(0);
    }

    /* -------------------------------- Configure drivers ---------------------------- */

    ethercat::devices::elmo::config::PolarityConfig left_wheels_pol_config = {
        .position_polarity = ethercat::devices::elmo::config::Polarity::Reversed,
        .velocity_polarity = ethercat::devices::elmo::config::Polarity::Reversed
    };

    // Invert polarity of left motors so that all wheels have the same 'positive' direction
    drivers[Wheels::RearLeft  ].write_polarity(left_wheels_pol_config);
    drivers[Wheels::FrontLeft ].write_polarity(left_wheels_pol_config);

    // Register input-data-handlers for all drivers
    drivers[Wheels::RearLeft  ].set_input_handler(make_driver_callback(Wheels::RearLeft  ));
    drivers[Wheels::RearRight ].set_input_handler(make_driver_callback(Wheels::RearRight ));
    drivers[Wheels::FrontLeft ].set_input_handler(make_driver_callback(Wheels::FrontLeft ));
    drivers[Wheels::FrontRight].set_input_handler(make_driver_callback(Wheels::FrontRight));

    RCLCPP_INFO_STREAM(node->get_logger(), "Configured [Base] driver");

    /* ----------------------------- Initialize subscribers -------------------------- */
            
    *make_subscriber_builder(setpoint_velocities_sub)
        .node(*(this->node))
        .name(SETPOINT_VELOCITIES_SUB_TOPIC_NAME)
        .qos(TOPIC_QUEUE_SIZE)
        .callback(*this, &BaseDriver::setpoint_velocities_callback);
    
    /* ----------------------------- Initialize publishers --------------------------- */
            
    *make_publisher_builder(joint_states_pub)
        .node(*(this->node))
        .name(JOINT_STATES_PUB_TOPIC_NAME)
        .qos(TOPIC_QUEUE_SIZE);
            
    *make_publisher_builder(encoders_pub)
        .node(*(this->node))
        .name(ENCODERS_PUB_TOPIC_NAME)
        .qos(TOPIC_QUEUE_SIZE);
            
    *make_publisher_builder(status_pub)
        .node(*(this->node))
        .name(STATUS_PUB_TOPIC_NAME)
        .qos(TOPIC_QUEUE_SIZE);
    
    /* ------------------------------ Initialize services ---------------------------- */

    *make_service_builder(enable_srv)
        .node(*(this->node))
        .name(ENABLE_SRV_TOPIC_NAME)
        .callback(*this, &BaseDriver::enable_callback);

    *make_service_builder(get_state)
        .node(*(this->node))
        .name(GET_STATE_SRV_TOPIC_NAME)
        .callback(*this, &BaseDriver::get_state_callback);

    *make_service_builder(reset_failure_srv)
        .node(*(this->node))
        .name(RESET_FAILURE_SRV_TOPIC_NAME)
        .callback(*this, &BaseDriver::reset_failure_callback);

    /* ---------------------------- Initialize TF interfaces ------------------------- */
    
    static_tf_broadcaster = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*(this->node));

    /* ----------------------- Provide static TF transformations --------------------- */

    geometry_msgs::msg::TransformStamped tf_model_transform_msg;

    // Prepare header of the robot's world-position's transformation message
    tf_model_transform_msg.header.stamp    = node->get_clock()->now();
    tf_model_transform_msg.header.frame_id = velmwheel::params::ROBOT_NAME;
    tf_model_transform_msg.child_frame_id  = "base_link";
    // Fill the body of the robot's world-position's transformation message
    tf_model_transform_msg.transform.translation.x = 0;
    tf_model_transform_msg.transform.translation.y = 0;
    tf_model_transform_msg.transform.translation.z = 0;
    // Set identity rotation
    tf_model_transform_msg.transform.rotation = tf2::toMsg(tf2::Quaternion::getIdentity());
    // Update robot's world-position's transformation
    static_tf_broadcaster->sendTransform(tf_model_transform_msg);
    
    /* ------------------------------------------------------------------------------- */
    
    RCLCPP_INFO_STREAM(node->get_logger(), "Registered ROS interfaces for [Base] driver");
}

/* ================================================================================================================================ */

} // End namespace velmwheel

/* ======================================================== Plugin includes ======================================================= */

#include <pluginlib/class_list_macros.hpp>

/* ======================================================== Plugins exports ======================================================= */

PLUGINLIB_EXPORT_CLASS(velmwheel::BaseDriver, velmwheel::EthercatSlaveDriver)

/* ================================================================================================================================ */
