/* ============================================================================================================================ *//**
 * @file       laser_driver.cpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Tuesday, 5th July 2022 3:06:46 am
 * @modified   Thursday, 7th July 2022 5:58:39 pm
 * @project    engineering-thesis
 * @brief      ROS2-based class implementing LIDAR sensor driver node for the Velmwheel's driveline
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

/* =========================================================== Includes =========================================================== */

// Standard includes
#include <regex>
// Libraries includes
#include "tf2_eigen/tf2_eigen.hpp"
// Private includes
#include "velmwheel/laser_driver.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace velmwheel {

/* ========================================================= Ctors & dtors ======================================================== */

LaserDriver::LaserDriver(const rclcpp::NodeOptions & options) : 
    rclcpp::Node(NODE_NAME, options)
{
    /* ----------------------------- Initialize parameters --------------------------- */

    auto hostname           = node_common::parameters::declare_parameter_and_get(*this, HOSTNAME_PARAM_DESCRIPTOR);
    auto devicename         = node_common::parameters::declare_parameter_and_get(*this, DEVICENAME_PARAM_DESCRIPTOR);
    auto publishing_mode    = node_common::parameters::declare_parameter_and_get(*this, PUBLISHING_MODE_PARAM_DESCRIPTOR);
    auto scan_frequency     = node_common::parameters::declare_parameter_and_get(*this, SCAN_FREQUENCY_PARAM_DESCRIPTOR);
    auto angular_resolution = node_common::parameters::declare_parameter_and_get(*this, ANGULAR_RESOLUTION_PARAM_DESCRIPTOR);
    auto use_ntp            = node_common::parameters::declare_parameter_and_get(*this, USE_NTP_PARAM_DESCRIPTOR);
    auto reference_frame    = node_common::parameters::declare_parameter_and_get(*this, REFERENCE_FRAME_PARAM_DESCRIPTOR);

    /* ----------------------------- Validate parameters ----------------------------- */

    // Check if non-empty hostname has been given
    if(not hostname.has_value() or hostname->empty())
        rclcpp::exceptions::InvalidParametersException("'hostname' parameter is invalid");
    // Check if non-empty devicename has been given
    if(not devicename.has_value() or devicename->empty())
        rclcpp::exceptions::InvalidParametersException("'devicename' parameter is invalid");

    // Check if valid publishing mode has been set
    if(auto &valid_values = PUBLISHING_MODE_PARAM_VALID_VALUES;
        std::find(valid_values.begin(), valid_values.end(), *publishing_mode) == valid_values.end()
    ) {
        std::stringstream err_msg;

        // Build error message
        err_msg << "[" << this->get_name() << "]" << "Invalid value of the " << PUBLISHING_MODE_PARAM_DESCRIPTOR.name
                << " parameter given " << "(" << *publishing_mode << ")";
        // Print an error
        RCLCPP_FATAL_STREAM(this->get_logger(), err_msg.str());
        // Throw an error
        throw rclcpp::exceptions::InvalidParametersException(err_msg.str().c_str());
    }

    // Check if valid scan frequency has been set
    if(auto &valid_values = SCAN_FREQUENCY_PARAM_VALID_VALUES;
        std::find(valid_values.begin(), valid_values.end(), *scan_frequency) == valid_values.end()
    ) {
        std::stringstream err_msg;

        // Build error message
        err_msg << "[" << this->get_name() << "]" << "Invalid value of the " << SCAN_FREQUENCY_PARAM_DESCRIPTOR.name
                << " parameter given " << "(" << *scan_frequency << ")";
        // Print an error
        RCLCPP_FATAL_STREAM(this->get_logger(), err_msg.str());
        // Throw an error
        throw rclcpp::exceptions::InvalidParametersException(err_msg.str().c_str());
    }

    // Check if valid scan resolution has been set
    if(auto &valid_values = ANGULAR_RESOLUTION_PARAM_VALID_VALUES;
        std::find(valid_values.begin(), valid_values.end(), *angular_resolution) == valid_values.end()
    ) {
        std::stringstream err_msg;

        // Build error message
        err_msg << "[" << this->get_name() << "]" << "Invalid value of the " << ANGULAR_RESOLUTION_PARAM_DESCRIPTOR.name
                << " parameter given " << "(" << *angular_resolution << ")";
        // Print an error
        RCLCPP_FATAL_STREAM(this->get_logger(), err_msg.str());
        // Throw an error
        throw rclcpp::exceptions::InvalidParametersException(err_msg.str().c_str());
    }

    /* -------------------------------- Parse parameters ----------------------------- */

    // Check if NTP is to be used
    this->use_ntp = *use_ntp;

    /// Construct the driver
    driver.emplace(this->get_logger(), std::string_view{*devicename});

    // Configure LIDAR
    configure_lidar(
        *hostname,
        *scan_frequency,
        *angular_resolution
    );

    // Run the LIDAR
    run_lidar();
    
    /* ----------------------------- Initialize publishers --------------------------- */

    // [Publisher] Create ROS publisher interface for broadcasting LIDAR measurements
    if(*publishing_mode == "separate" or *publishing_mode == "both") {
        *node_common::communication::make_publisher_builder(pub)
            .node(*this)
            .name(SCAN_TOPIC_NAME)
            .qos(TOPIC_QUEUE_SIZE);
    }

    // [Publisher] Create ROS publisher interface for broadcasting LIDAR measurements (comon for all LIDARs)
    if(*publishing_mode == "common" or *publishing_mode == "both") {
        *node_common::communication::make_publisher_builder(common_pub)
            .node(*this)
            .name(COMMON_SCAN_TOPIC_NAME)
            .qos(TOPIC_QUEUE_SIZE);
    }

    // Initialize TF2 broadcaster
    static_tf_broadcaster = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

    /* ----------------------------- Initialize callbacks ---------------------------- */

    // Initialize subscriber of the topic broadcasting encoders measurements
    *node_common::node::make_wall_timer_builder(timer)
        .node(*this)
        .period(std::chrono::milliseconds{ 1000 / (*scan_frequency) })
        .callback(*this, &LaserDriver::measurement_callback);

    /* ---------------------- Publish static transformations --------------------- */

    // COmpute name of the TF base frame
    base_tf_frame = *reference_frame;

    geometry_msgs::msg::TransformStamped lidar_transform_msg;

    // Prepare header of the robot's world-position's transformation message
    lidar_transform_msg.header.stamp    = this->get_clock()->now();
    lidar_transform_msg.header.frame_id = std::string(this->get_name()) + "_core";
    // Fill the body of the robot's world-position's transformation message
    lidar_transform_msg.child_frame_id          = base_tf_frame;
    lidar_transform_msg.transform.translation.x = 0;
    lidar_transform_msg.transform.translation.y = 0;
    lidar_transform_msg.transform.translation.z = 0;
    lidar_transform_msg.transform.rotation.x    = tf2::Quaternion::getIdentity().getX();
    lidar_transform_msg.transform.rotation.y    = tf2::Quaternion::getIdentity().getY();
    lidar_transform_msg.transform.rotation.z    = tf2::Quaternion::getIdentity().getZ();
    lidar_transform_msg.transform.rotation.w    = tf2::Quaternion::getIdentity().getW();
    // Update robot's world-position's transformation
    static_tf_broadcaster->sendTransform(lidar_transform_msg);
 
    /* ------------------------------------------------------------------------------- */
    
    node_common::node::print_hello(*this);
}


LaserDriver::~LaserDriver() {
    node_common::node::print_goodbye(*this);   
}

/* ================================================================================================================================ */

} // End namespace velmwheel

/* ======================================================== Nodes' registry ======================================================= */

RCLCPP_COMPONENTS_REGISTER_NODE(velmwheel::LaserDriver)

/* ================================================================================================================================ */

