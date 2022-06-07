/* ============================================================================================================================ *//**
 * @file       lidar_gazebo.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 9th March 2022 4:41:08 pm
 * @modified   Wednesday, 25th May 2022 11:56:39 pm
 * @project    engineering-thesis
 * @brief      Implementation of the dedicated LIDAR plugin for the Gazebo simulation of the WUT Velmobil robot
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __VELMWHEEL_GAZEBO_LIDAR_GAZEBO_H__
#define __VELMWHEEL_GAZEBO_LIDAR_GAZEBO_H__

/* =========================================================== Includes =========================================================== */

// System includes
#include <functional>
#include <string>
#include <iostream>
// Gazebo includes
#include <gazebo/gazebo.hh>
#include <gazebo_ros/node.hpp>
#include <gazebo/sensors/sensors.hh>
// ROS includes
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
// TF2 includes
#include <tf2_ros/static_transform_broadcaster.h>
// Private includes
#include "node_common/parameters.hpp"
#include "node_common/communication.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace gazebo { 

/* ============================================================= Nodes ============================================================ */

/**
 * @brief Implementation of the dedicated LIDAR plugin for the Gazebo simulation of the WUT
 *    Velmobil robot
 */
class Lidar : public SensorPlugin {

public: /* ------------------------------------------------ Node's parameters ----------------------------------------------------- */
    
    /// Description of the parameter determining topics that the plugin is publishing to
    static constexpr node_common::parameters::ParamDescriptor<std::string> PUBLISHING_MODE_PARAM_DESCRIPTOR {
        .name           = "publishing_mode",
        .read_only      = true,
        .dynamic_typing = false,
        .default_value  = "separate",
        .description    = "Publishing mode of the node indicating what topics it publishes to. Possible values are: \n"
                          "  'separate' - publishes measurements to ~/scan topic \n"
                          "  'common'   - publishes measurements to lidars/scan topic \n"
                          "  'both'     - publishes measurements to both topics"
    };

    /// Valid values of the node's parameter determining topics that the plugin is publishing to
    static constexpr std::array PUBLISHING_MODE_PARAM_VALID_VALUES { "separate", "common", "both" };

public: /* ------------------------------------------------ Topic's parameters ---------------------------------------------------- */

    /// Size of the topics' queue
    static constexpr std::size_t TOPIC_QUEUE_SIZE = 1000;

    /// Unqualified name of the topic that measurement are published onto
    static constexpr auto SCAN_TOPIC_NAME = "~/scan";
    /// Unqualified name of the topic that measurement are published onto (common for all LIDARs)
    static constexpr auto COMMON_SCAN_TOPIC_NAME = "lidars/scan";
    /// Scheme of the name of the base TF frame for the scan
    static constexpr auto SCAN_BASE_FRAME = "<node_name>_scan";
    
public: /* ----------------------------------------------- Public ctors & dtors --------------------------------------------------- */

    /**
     * @brief Construct a new Plugin object
     */
    Lidar() = default;

private: /* -------------------------------------------- Gazebo-specific methods -------------------------------------------------- */

	/**
	 * @brief On-load Gazebo initialization routine of the plugin
     * @param parent 
     *    pointer to the Gazebo Sensor model (injected by the Gazebo)
     * @param sdf
     *    pointer to the SDF <plugin> element of the sensor's description (injected by the Gazebo)
	 */
	void Load(sensors::SensorPtr parent, sdf::ElementPtr sdf);

	/**
	 * @brief On simulation update transfers LIDAR measurements to the ROS topic
	 */
	void OnUpdate();

private: /* ------------------------------------------------- ROS interfaces ------------------------------------------------------ */

    /// ROS node
    gazebo_ros::Node::SharedPtr node;
	/// Publisher interface to broadcast LIDAR measurements on
	rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub{ nullptr };
	/// Publisher interface to broadcast LIDAR measurements on (common for all LIDARs)
	rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr common_pub{ nullptr };

    /// Name of the associated base TF frame
    std::string base_tf_frame;
    /// TF2 publishing object for broadcastign position of the LIDAR
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster;
	
private: /* -------------------------------------------------- Node's state ------------------------------------------------------- */

	/// Reference to the Gazebo sensor
	sensors::RaySensorPtr sensor;
	/// Reference to the on-update interface of Gazebo simulation
	event::ConnectionPtr update_connection;

};

/* ================================================================================================================================ */

} // End namespace gazebo

#endif
