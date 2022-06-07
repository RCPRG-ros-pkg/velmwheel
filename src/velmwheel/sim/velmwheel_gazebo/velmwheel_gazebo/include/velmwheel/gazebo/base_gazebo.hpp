/* ============================================================================================================================ *//**
 * @file       base_gazebo.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 9th March 2022 4:41:08 pm
 * @modified   Wednesday, 25th May 2022 11:55:34 pm
 * @project    engineering-thesis
 * @brief      Implementation of the dedicated Base plugin for the Gazebo simulation of the WUT Velmwheel robot
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __VELMWHEEL_GAZEBO_VELMWHEEL_BASE_H__
#define __VELMWHEEL_GAZEBO_VELMWHEEL_BASE_H__

/* ============================================================ Macros ============================================================ */

// Define default plugin implementation type
#if !defined(PLUGIN_TYPE_REAL) && !defined(PLUGIN_TYPE_IDEAL)
#define PLUGIN_TYPE_REAL
#endif

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
#include "sensor_msgs/msg/joint_state.hpp"
// TF2 includes
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
// Private includes
#include "velmwheel_msgs/msg/encoders.hpp"
#include "velmwheel_msgs/msg/encoders_stamped.hpp"
#include "velmwheel_msgs/msg/wheel.hpp"
#include "velmwheel_msgs/msg/wheel_enum.hpp"
#include "velmwheel_msgs/msg/wheels.hpp"
#include "velmwheel/params.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace gazebo { 

/* ============================================================= Nodes ============================================================ */

/**
 * @brief Implementation of the dedicated Base plugin for the Gazebo simulation of the WUT
 *    Velmwheel robot
 */
class Base : public ModelPlugin {

public: /* ------------------------------------------------ Topic's parameters ---------------------------------------------------- */

    /// Size of the topics' queue
    static constexpr std::size_t TOPIC_QUEUE_SIZE = 1000;

    /// Unqualified name of the topic subscribed by the plugin to get setpoint angular velocities for robot' wheels
    static constexpr auto SETPOINT_VELOCITIES_TOPIC_NAME = "base/controls";

    /// Unqualified name of the topic used by the plugin to broadcast joints' states (repeats information on '~/encoders' topic in different form)
    static constexpr auto JOINT_STATES_TOPIC_NAME = "base/joint_states";
    /// Unqualified name of the topic used by the plugin to broadcast measurements of the robot wheels' encoders
    static constexpr auto ENCODERS_TOPIC_NAME = "base/encoders";

public: /* ----------------------------------------------- Public ctors & dtors --------------------------------------------------- */

    /**
     * @brief Construct a new Plugin object
     */
    Base() = default;

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
     * @brief Callback method of the ROS2 subscriber interface setting setpoint angular
     *    velocities of robot wheels at arrival of the setpoint message on the subscribed 
     *    topic
     * 
     * @param msg 
     *    message containing setpoint angular velocities for wheel in [rad/s]
     */
    void SetVelocitySetpoints(const velmwheel_msgs::msg::Wheels &msg);

private: /* ----------------------------------------------- Auxiliary methods ----------------------------------------------------- */

    /**
     * @brief Implementation-specific on-load routine
     * 
     * @param wheel_rr
     *    reference to the rear-right wheel link 
     * @param wheel_rl
     *    reference to the rear-left wheel link 
     * @param wheel_fr
     *    reference to the front-right wheel link 
     * @param wheel_fl
     *    reference to the front-left wheel link 
     */
    inline void OnLoadImpl(
        gazebo::physics::LinkPtr &wheel_rr,
        gazebo::physics::LinkPtr &wheel_rl,
        gazebo::physics::LinkPtr &wheel_fr,
        gazebo::physics::LinkPtr &wheel_fl
    );

    /**
     * @brief Implementation-specific on-update routine
     * 
     * @param model_pose 
     *    reference to the current model's pose
     * @param model_lin_vel 
     *    reference to the current model's linear velocity
     * @param model_rot_vel 
     *    reference to the current model's angular velocity
     * @param now 
     *    reference to the current time
     */
    inline velmwheel_msgs::msg::Encoders OnUpdateImpl(const ignition::math::Pose3d &model_pose);

private: /* ------------------------------------------------- ROS interfaces ------------------------------------------------------ */

    /// ROS node
    gazebo_ros::Node::SharedPtr node;

	/// Subscriber interface used to read current velocity setpoint
	rclcpp::Subscription<velmwheel_msgs::msg::Wheels>::SharedPtr setpoint_velocities_sub;

	/// Publisher interface used to publish current joint states
	rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub;
	/// Publisher interface used to publish current measurements of encoders
	rclcpp::Publisher<velmwheel_msgs::msg::EncodersStamped>::SharedPtr encoders_pub;
    
    /// TF2 publishing object for broadcastign output frames
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster;

private: /* -------------------------------------------------- Node's state ------------------------------------------------------- */

	/// Pointer to the physical model
	physics::ModelPtr model;
	/// Reference to the on-update interface of Gazebo simulation
	event::ConnectionPtr update_connection;
	/// Common prefix of links in the model
	std::string link_prefix;

	/// Model of the motor of the read right wheel
	physics::JointPtr motor_rr;
	/// Model of the motor of the read left wheel
	physics::JointPtr motor_rl;
	/// Model of the motor of the front right wheel
	physics::JointPtr motor_fr;
	/// Model of the motor of the front left wheel
	physics::JointPtr motor_fl;

	/// Current setpoint velocity vector
    velmwheel_msgs::msg::Wheels current_controls_setpoint;
	
#if defined(PLUGIN_TYPE_REAL)

	/// Model of the collision of the read right wheel
    physics::CollisionPtr wheel_rr_collision;
	/// Model of the collision of the read left wheel
    physics::CollisionPtr wheel_rl_collision;
	/// Model of the collision of the front right wheel
    physics::CollisionPtr wheel_fr_collision;
	/// Model of the collision of the front left wheel
    physics::CollisionPtr wheel_fl_collision;

#endif

};

/* ================================================================================================================================ */

} // End namespace gazebo

#endif
