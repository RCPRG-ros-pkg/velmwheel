/* ============================================================================================================================ *//**
 * @file       imu_gazebo.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 9th March 2022 4:41:08 pm
 * @modified   Monday, 30th May 2022 11:38:55 pm
 * @project    engineering-thesis
 * @brief      Implementation of the dedicated IMU plugin for the Gazebo simulation of the WUT velmobil robot
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __VELMWHEEL_GAZEBO_IMU_GAZEBO_H__
#define __VELMWHEEL_GAZEBO_IMU_GAZEBO_H__

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
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

/* ========================================================== Namespaces ========================================================== */

namespace gazebo { 

/* ============================================================= Nodes ============================================================ */

/**
 * @brief Implementation of the dedicated IMU plugin for the Gazebo simulation of the WUT
 *    velmobil robot
 */
class Imu : public SensorPlugin {

public: /* ------------------------------------------------ Topic's parameters ---------------------------------------------------- */

    /// Size of the topics' queue
    static constexpr std::size_t TOPIC_QUEUE_SIZE = 1000;
    /// Name of the TF reference frame
    static constexpr auto BASE_TF_FRAME = "imu_centre";

    /// Unqualified name of the topic that measurement are published onto
    static constexpr auto OUT_TOPIC_NAME = "imu/out";
    
public: /* ----------------------------------------------- Public ctors & dtors --------------------------------------------------- */

    /**
     * @brief Construct a new Plugin object
     */
    Imu() = default;

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
	 * @brief On simulation update transfers IMU measurements to the ROS topic
	 */
	void OnUpdate();

private: /* ------------------------------------------------- ROS interfaces ------------------------------------------------------ */

    /// ROS node
    gazebo_ros::Node::SharedPtr node;
	/// Publisher interface to broadcast IMU measurements on
	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub;

private: /* -------------------------------------------------- Node's state ------------------------------------------------------- */
	
	/// Reference to the Gazebo sensor
	sensors::ImuSensorPtr sensor;
	/// Reference to the on-update interface of Gazebo simulation
	event::ConnectionPtr update_connection;

};

/* ================================================================================================================================ */

} // End namespace gazebo

#endif
