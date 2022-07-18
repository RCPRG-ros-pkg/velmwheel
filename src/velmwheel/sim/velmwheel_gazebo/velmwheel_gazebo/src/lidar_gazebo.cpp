/* ============================================================================================================================ *//**
 * @file       lidar_gazebo.cpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Tuesday, 8th March 2022 5:16:22 pm
 * @modified   Tuesday, 5th July 2022 3:19:36 am
 * @project    engineering-thesis
 * @brief      Implementation of the LIDAR sensor's Gazebo plugin for WUT Velmobil robot
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

/* =========================================================== Includes =========================================================== */

// Standard includes
#include <regex>
#include <ignition/math/Quaternion.hh>
// Private includes
#include "velmwheel/gazebo/lidar_gazebo.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace gazebo { 

/* ============================================================= Nodes ============================================================ */

void Lidar::Load(sensors::SensorPtr parent, sdf::ElementPtr sdf) {
        
    using namespace std::literals;

    /* ------------------------- Initialize ROS interface ------------------------ */
    
    // Create ROS node
    node = gazebo_ros::Node::Get(sdf);

    // Declare parameter determining topics that the plugin is publishing to
    auto publishing_mode = node_common::parameters::declare_parameter_and_get(*node, PUBLISHING_MODE_PARAM_DESCRIPTOR);

    // Check if valid value has been set
    if(auto &valid_values = PUBLISHING_MODE_PARAM_VALID_VALUES;
        std::find(valid_values.begin(), valid_values.end(), *publishing_mode) == valid_values.end()
    ) {

        std::stringstream err_msg;

        // Build error message
        err_msg << "[" << node->get_name() << "]" << "Invalid value of the " << PUBLISHING_MODE_PARAM_DESCRIPTOR.name
                << " parameter given " << "(" << *publishing_mode << ")";
        // Print an error
        RCLCPP_FATAL_STREAM(node->get_logger(), err_msg.str());
        // Throw an error
        throw std::runtime_error(err_msg.str().c_str());
    }

    // [Publisher] Create ROS publisher interface for broadcasting LIDAR measurements
    if(*publishing_mode == "separate" or *publishing_mode == "both") {
        *node_common::communication::make_publisher_builder(pub)
            .node(*node)
            .name(SCAN_TOPIC_NAME)
            .qos(TOPIC_QUEUE_SIZE);
    }

    // [Publisher] Create ROS publisher interface for broadcasting LIDAR measurements (comon for all LIDARs)
    if(*publishing_mode == "common" or *publishing_mode == "both") {
        *node_common::communication::make_publisher_builder(common_pub)
            .node(*node)
            .name(COMMON_SCAN_TOPIC_NAME)
            .qos(TOPIC_QUEUE_SIZE);
    }

    // Initialize TF2 broadcaster
    static_tf_broadcaster = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*node);
    
    /* ------------------------ Initialize Gazebo interface ---------------------- */

    // Get pointer to the sensor plugin from the upstream
    if(sensor = std::dynamic_pointer_cast<sensors::RaySensor>(parent); !sensor)
        RCLCPP_FATAL_STREAM(node->get_logger(), "[lidar_gazebo] Failed to soruce reference to the Gazebo sensor plugin");

    // Connect on-update action to the plugin
    update_connection = sensor->ConnectUpdated(std::bind(&Lidar::OnUpdate, this));
    // Activate the sensor
    sensor -> SetActive(true);

    /* ---------------------- Publish static transformations --------------------- */

    // COmpute name of the TF base frame
    base_tf_frame = std::regex_replace(SCAN_BASE_FRAME, std::regex("<node_name>"), node->get_name());

    geometry_msgs::msg::TransformStamped lidar_transform_msg;

    // Prepare header of the robot's world-position's transformation message
    lidar_transform_msg.header.stamp    = node->get_clock()->now();
    lidar_transform_msg.header.frame_id = std::string(node->get_name()) + "_core";
    // Fill the body of the robot's world-position's transformation message
    lidar_transform_msg.child_frame_id          = base_tf_frame;
    lidar_transform_msg.transform.translation.x = 0;
    lidar_transform_msg.transform.translation.y = 0;
    lidar_transform_msg.transform.translation.z = 0;
    lidar_transform_msg.transform.rotation.x    = ignition::math::Quaternion<double>::Identity.X();
    lidar_transform_msg.transform.rotation.y    = ignition::math::Quaternion<double>::Identity.Y();
    lidar_transform_msg.transform.rotation.z    = ignition::math::Quaternion<double>::Identity.Z();
    lidar_transform_msg.transform.rotation.w    = ignition::math::Quaternion<double>::Identity.W();
    // Update robot's world-position's transformation
    static_tf_broadcaster->sendTransform(lidar_transform_msg);
    
    /* --------------------------------------------------------------------------- */

    // Log info message
    RCLCPP_INFO_STREAM(node->get_logger(), "Running 'lidar_gazebo' plugin ('" << *publishing_mode << "' publishing model)...");
}

void Lidar::OnUpdate() {

    // Get current system time-point
    auto now = node->get_clock()->now();
    
    sensor_msgs::msg::LaserScan msg;
    
    // Fill the message's header
    msg.header.frame_id      = base_tf_frame;
    msg.header.stamp.sec     = sensor->LastMeasurementTime().sec;
    msg.header.stamp.nanosec = sensor->LastMeasurementTime().nsec;
    // Fill message's body
    msg.angle_min       = sensor->AngleMin().Radian();
    msg.angle_max       = sensor->AngleMax().Radian();
    msg.angle_increment = sensor->AngleResolution();
    msg.time_increment  = 0;
    msg.scan_time       = 1.0 / sensor->UpdateRate();
    msg.range_max       = sensor->RangeMax();
    msg.range_min       = sensor->RangeMin();

    // Resize the output vectors
    msg.ranges.reserve(sensor->RangeCount());
    msg.intensities.reserve(sensor->RangeCount());
    // Fill message's output samples
    for(int i = 0; i < sensor->RangeCount(); i++) {
        msg.ranges.push_back(sensor->Range(i));
        msg.intensities.push_back(sensor->Retro(i));
    }
        
    // Publish the message
    if(pub) 
        pub->publish(msg);
    // Publish the message (common topic)
    if(common_pub) 
        common_pub->publish(msg);
}

/* ================================================================================================================================ */

// Register class as Gazebo plugin
GZ_REGISTER_SENSOR_PLUGIN(Lidar)

/* ================================================================================================================================ */

} // End namespace gazebo
