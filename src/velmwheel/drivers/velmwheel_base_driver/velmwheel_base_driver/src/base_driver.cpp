/* ============================================================================================================================ *//**
 * @file       base_driver.cpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 28th April 2022 9:24:14 pm
 * @modified   Monday, 13th June 2022 10:52:21 pm
 * @project    engineering-thesis
 * @brief      Definition of the driver plugin class for the servodriver EtherCAT slaves of the WUT Velmwheel robot
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

/* =========================================================== Includes =========================================================== */

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

    // Return device name
    return std::vector<std::string>{ 
        *rear_left_wheel_name,
        *rear_right_wheel_name,
        *front_left_wheel_name,
        *front_right_wheel_name
    };

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

    // Construct driver's implementation ( must match order of entries of vector returned from @ref initialize )
    drivers.emplace_back(*slaves[Wheels::RearLeft  ], config);
    drivers.emplace_back(*slaves[Wheels::RearRight ], config);
    drivers.emplace_back(*slaves[Wheels::FrontLeft ], config);
    drivers.emplace_back(*slaves[Wheels::FrontRight], config);

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

    // Register input-data-handlers for all drivers
    drivers[Wheels::RearLeft  ].set_input_handler(make_driver_callback(Wheels::RearLeft  ));
    drivers[Wheels::RearRight ].set_input_handler(make_driver_callback(Wheels::RearRight ));
    drivers[Wheels::FrontLeft ].set_input_handler(make_driver_callback(Wheels::FrontLeft ));
    drivers[Wheels::FrontRight].set_input_handler(make_driver_callback(Wheels::FrontRight));

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

    *make_service_builder(command_srv)
        .node(*(this->node))
        .name(COMMAND_SRV_TOPIC_NAME)
        .callback(*this, &BaseDriver::command_callback);

    *make_service_builder(reset_failure_srv)
        .node(*(this->node))
        .name(RESET_FAILURE_SRV_TOPIC_NAME)
        .callback(*this, &BaseDriver::reset_failure_callback);
    
}

/* ================================================================================================================================ */

} // End namespace velmwheel

/* ======================================================== Plugin includes ======================================================= */

#include <pluginlib/class_list_macros.hpp>

/* ======================================================== Plugins exports ======================================================= */

PLUGINLIB_EXPORT_CLASS(velmwheel::BaseDriver, velmwheel::EthercatSlaveDriver)

/* ================================================================================================================================ */
