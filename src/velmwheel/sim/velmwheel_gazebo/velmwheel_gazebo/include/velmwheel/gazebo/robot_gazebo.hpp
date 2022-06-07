/* ============================================================================================================================ *//**
 * @file       robot_gazebo.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 9th March 2022 4:41:08 pm
 * @modified   Wednesday, 25th May 2022 11:57:03 pm
 * @project    engineering-thesis
 * @brief      
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __VELMWHEEL_GAZEBO_VELMWHEEL_ROBOT_H__
#define __VELMWHEEL_GAZEBO_VELMWHEEL_ROBOT_H__

/* =========================================================== Includes =========================================================== */

// System includes
#include <functional>
#include <string>
#include <iostream>
// Gazebo includes
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
// ROS includes
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
// TF2 includes
#include "tf2_ros/transform_broadcaster.h"
// Private includes
#include "velmwheel_gazebo_msgs/srv/friction_config.hpp"
#include "velmwheel_gazebo_msgs/srv/inertia_config.hpp"
#include "velmwheel/params.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace gazebo { 

/* ============================================================= Nodes ============================================================ */

/**
 * @brief Implementation of the dedicated Robot plugin for the Gazebo simulation of the WUT
 *    Velmwheel robot broadcasting iiformations about state of the robot in the simulated world
 */
class Robot : public ModelPlugin {

public: /* ------------------------------------------------ Topic's parameters ---------------------------------------------------- */

    /// Size of the topics' queue
    static constexpr std::size_t TOPIC_QUEUE_SIZE = 1000;
    
    /// Unqualified name of the topic used by the plugin to broadcast real position of the robot in the ismulation
    static constexpr auto SIM_POSE_TOPIC_NAME = "~/pose";
    /// Unqualified name of the topic used by the plugin to broadcast real velocity of the robot in the ismulation
    static constexpr auto SIM_VELOCITY_TOPIC_NAME = "~/velocity";

    /// Unqualified name of the service shared by the plugin to set friction coefficients of robot wheel's rollers at runtime
    static constexpr auto SET_ROLLS_FRICTIONS_TOPIC_NAME = "~/set_rollers_friction";
    /// Unqualified name of the service shared by the plugin to set inertia of robot components at runtime
    static constexpr auto SET_INERTIA_TOPIC_NAME = "~/set_inertia";
    
public: /* ----------------------------------------------- Public ctors & dtors --------------------------------------------------- */

    /**
     * @brief Construct a new Plugin object
     */
    Robot() = default;

private: /* -------------------------------------------- Gazebo-specific methods -------------------------------------------------- */

	/**
	 * @brief On-load Gazebo initialization routine of the plugin
     * @param parent 
     *    pointer to the Gazebo Model model (injected by the Gazebo)
     * @param sdf
     *    pointer to the SDF <plugin> element of the sensor's description (injected by the Gazebo)
	 */
	void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);

	/**
	 * @brief Gazebo callback called on each initialization of the update of the world's frame 
	 */
	void OnUpdate();

private: /* ------------------------------------------------ Callback methods ----------------------------------------------------- */

    /**
     * @brief Callback method to the ROS2 service providing a runtime mechanis for setting
     *    friction coefficients of the rollers of robot's wheels
     * 
     * @param req 
     *    request structure containing coefficients to be set
     * @param res 
     *    reference to the response structure (unused) 
     */
    void SetFriction(
        const velmwheel_gazebo_msgs::srv::FrictionConfig::Request::SharedPtr req,
        velmwheel_gazebo_msgs::srv::FrictionConfig::Response::SharedPtr res
    );

    /**
     * @brief Callback method to the ROS2 service providing a runtime mechanis for setting
     *    inertia momemntum of robot's components
     * 
     * @param req 
     *    request structure containing inertia to be set
     * @param res 
     *    reference to the response structure (unused) 
     */
    void SetInertia(
        const velmwheel_gazebo_msgs::srv::InertiaConfig::Request::SharedPtr req,
        velmwheel_gazebo_msgs::srv::InertiaConfig::Response::SharedPtr res
    );

private: /* ------------------------------------------------- ROS interfaces ------------------------------------------------------ */

    /// ROS node
    gazebo_ros::Node::SharedPtr node;
    
	/// Publisher interface used to publish current position of the robot (actual value taken from the physical engine)
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
	/// Publisher interface used to publish current velocity of the robot (actual value taken from the physical engine)
	rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub;

	/// Service interface used to set friction coefficients of the wheel's rollers
	rclcpp::Service<velmwheel_gazebo_msgs::srv::FrictionConfig>::SharedPtr set_rolls_frictions_srv;
	/// Service interface used to set inertia of robot's elements
	rclcpp::Service<velmwheel_gazebo_msgs::srv::InertiaConfig>::SharedPtr set_inertia_srv;
    
    /// Name of the TF frame associated with the published messages
    const std::string tf_frame_name { std::string("sim_") + velmwheel::params::ROBOT_NAME };
    /// TF2 publishing object for broadcastign output frames
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

private: /* -------------------------------------------------- Node's state ------------------------------------------------------- */

	/// Pointer to the physical model
	physics::ModelPtr model;
	/// Reference to the on-update interface of Gazebo simulation
	event::ConnectionPtr update_connection;
	/// Common prefix of links in the model
	std::string link_prefix;
	
	/// Model of the collision of the read right wheel
    physics::CollisionPtr wheel_rr_collision;
	/// Model of the collision of the read left wheel
    physics::CollisionPtr wheel_rl_collision;
	/// Model of the collision of the front right wheel
    physics::CollisionPtr wheel_fr_collision;
	/// Model of the collision of the front left wheel
    physics::CollisionPtr wheel_fl_collision;

};

/* ================================================================================================================================ */

} // End namespace gazebo

#endif
