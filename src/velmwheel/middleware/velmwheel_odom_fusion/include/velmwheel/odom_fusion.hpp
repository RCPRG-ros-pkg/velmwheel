/* ============================================================================================================================ *//**
 * @file       bias_estimator.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 16th March 2022 4:37:20 pm
 * @modified   Thursday, 26th May 2022 1:03:50 am
 * @project    engineering-thesis
 * @brief      Declaration of the ROS2 node class implementing Kalman-Filter-based processing mechanism for filtering odometry data
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __VELMWHEEL_ODOM_FUSION_H__
#define __VELMWHEEL_ODOM_FUSION_H__

/* =========================================================== Includes =========================================================== */

// System includes
#include <string>
#include <fstream>
// Math includes
#include "tf2/LinearMath/Quaternion.h"
// ROS includes
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rclcpp_components/register_node_macro.hpp"
// Standard messages
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
// TF2 includes
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_msgs/msg/tf_message.hpp"
// Robot-localization includes
#include "robot_localization/ekf.hpp"
// Private includes
#include "node_common/parameters.hpp"
#include "node_common/communication.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace velmwheel {

/* ============================================================= Node ============================================================= */

/**
 * @brief ROS2 node class providing Kalman-Filter-based processing mechanism responsible 
 *    for fusion of (both encoder- and LIDAR-based) odometry data of the WUT Velmwheel
 *    robot using utilities provided by robot_localization package
 * @details Filter utilizes model of the process (i.e. model of the robot's pose) based
 *    on the following 15D state vector:
 * 
 *    \f[
 *        x[n] = \begin{bmatrix}
 *                  x   \\
 *                  y   \\
 *                  z   \\
 *                  ɑ   \\
 *                  β   \\
 *                  θ   \\
 *                  v⃗_x \\
 *                  v⃗_y \\
 *                  v⃗_z \\
 *                  ω⃗_ɑ \\
 *                  ω⃗_β \\
 *                  ω⃗_θ \\
 *                  a⃗_x \\
 *                  a⃗_y \\
 *                  a⃗_z
 *               \end{bmatrix}
 *               \begin{matrix}
 *                  \textrm{x position}     \\
 *                  \textrm{y position}     \\
 *                  \textrm{z position}     \\
 *                  \textrm{roll}           \\
 *                  \textrm{pitch}          \\
 *                  \textrm{yaw}            \\
 *                  \textrm{x velocity}     \\
 *                  \textrm{y velocity}     \\
 *                  \textrm{z velocity}     \\
 *                  \textrm{roll velocity}  \\
 *                  \textrm{pitch velocity} \\
 *                  \textrm{yaw velocity}   \\
 *                  \textrm{x acceleration} \\
 *                  \textrm{y acceleration} \\
 *                  \textrm{z acceleration}
 *               \end{matrix}
 *    \f]
 * 
 *    and 12D measurements vector:
 * 
 *    \f[
 *        z[n] = \begin{bmatrix}
 *                  x    \\
 *                  y    \\
 *                  -    \\
 *                  -    \\
 *                  -    \\
 *                  θ    \\
 *                  v⃗_x \\
 *                  v⃗_y \\
 *                  -    \\
 *                  -    \\
 *                  -    \\
 *                  ω⃗_θ
 *               \end{bmatrix}
 *               \begin{matrix}
 *                  \textrm{x position} \\
 *                  \textrm{y position} \\
 *                                      \\
 *                                      \\
 *                                      \\
 *                  \textrm{yaw}        \\
 *                  \textrm{x velocity} \\
 *                  \textrm{y velocity} \\
 *                                      \\
 *                                      \\
 *                                      \\
 *                  \textrm{yaw velocity}
 *               \end{matrix}
 *    \f]
 * 
 * @note '-' symbols in the z[n] vector means that the measurement isn't actually provided to the
 *    filtering algorithm. The 12D vector has been given to show what state variables are actually
 *    measured when compared to the x[n] vector 
 */
class RCLCPP_PUBLIC OdomFusion : public rclcpp::Node {

public: /* -------------------------------------------------- Node's traits ------------------------------------------------------- */

    /// Name of the node
    static constexpr auto NODE_NAME = "odom_fusion";

public: /* ------------------------------------------------- Node's constants ----------------------------------------------------- */

    /// Size of the state vector (x) describing the system
    static constexpr auto STATE_VECTOR_SIZE = 15;
    /// Size of the measurement vector (z) of thee system
    static constexpr auto MEASUREMENT_VECTOR_SIZE = 12;

    /// Named enumeration of the components of the system's state vector
    enum StateComponent: unsigned {
        StateX        = 0,  /**< \f$x\f$    (x position)     */
        StateY        = 1,  /**< \f$y\f$    (y position)     */
        StateZ        = 2,  /**< \f$z\f$    (z position)     */
        StateRoll     = 3,  /**< \f$ɑ\f$    (roll)           */
        StatePitch    = 4,  /**< \f$β\f$    (pitch)          */
        StateYaw      = 5,  /**< \f$θ\f$    (yaw)            */
        StateVelX     = 6,  /**< \f$v⃗_x\f$ (x velocity)     */
        StateVelY     = 7,  /**< \f$v⃗_y\f$ (y velocity)     */
        StateVelZ     = 8,  /**< \f$v⃗_z\f$ (z velocity)     */
        StateVelRoll  = 9,  /**< \f$ω⃗_ɑ\f$ (roll velocity)  */
        StateVelPitch = 10, /**< \f$ω⃗_β\f$ (pitch velocity) */
        StateVelYaw   = 11, /**< \f$ω⃗_θ\f$ (yaw velocity)   */
        StateAccX     = 12, /**< \f$a⃗_x\f$ (x acceleration) */
        StateAccY     = 13, /**< \f$a⃗_y\f$ (y acceleration) */
        StateAccZ     = 14, /**< \f$a⃗_z\f$ (z acceleration) */
    };

    /**
     * @brief Vector of booleans indicating which components of the robot's velocity (twist) 
     *    vector can be directly controled. Flags given in the following order
     * 
     *       * [0] \f$v⃗_x\f$ (x velocity)
     *       * [1] \f$v⃗_y\f$ (y velocity)
     *       * [2] \f$v⃗_z\f$ (z velocity)
     *       * [3] \f$ω⃗_x\f$ (roll velocity)
     *       * [4] \f$ω⃗_y\f$ (pitch velocity)
     *       * [5] \f$ω⃗_z\f$ (yaw velocity)
     * 
     * @note Units of the given values are SI units for the given dimension.
     */
    static constexpr std::array<bool, robot_localization::TWIST_SIZE> CONTROL_AFFECTION_VECTOR { true, true, false, false, false, true };

    /// Vector indicating measurements of what state variables are directly published on the odometry input topic
    static constexpr std::array<bool, MEASUREMENT_VECTOR_SIZE> ODOM_MEASURED_STATE {
        false, /**< \f$x\f$ (x position)        */
        false, /**< \f$y\f$ (y position)        */
        false, /**< \f$z\f$ (z position)        */
        false, /**< \f$ɑ\f$ (roll)              */
        false, /**< \f$β\f$ (pitch)             */
        false, /**< \f$θ\f$ (yaw)               */
        true,  /**< \f$v⃗_x\f$ (x velocity)     */
        true,  /**< \f$v⃗_y\f$ (y velocity)     */
        false, /**< \f$v⃗_z\f$ (z velocity)     */
        false, /**< \f$ω⃗_ɑ\f$ (roll velocity)  */
        false, /**< \f$ω⃗_β\f$ (pitch velocity) */
        true   /**< \f$ω⃗_θ\f$ (yaw velocity)   */
    };

    /// Vector indicating measurements of what state variables are directly published on the laser odometry input topic
    static constexpr std::array<bool, MEASUREMENT_VECTOR_SIZE> LASER_ODOM_MEASURED_STATE {
        true,  /**< \f$x\f$ (x position)        */ 
        true,  /**< \f$y\f$ (y position)        */ 
        false, /**< \f$z\f$ (z position)        */ 
        false, /**< \f$ɑ\f$ (roll)              */ 
        false, /**< \f$β\f$ (pitch)             */ 
        true,  /**< \f$θ\f$ (yaw)               */ 
        false, /**< \f$v⃗_x\f$ (x velocity)     */ 
        false, /**< \f$v⃗_y\f$ (y velocity)     */ 
        false, /**< \f$v⃗_z\f$ (z velocity)     */ 
        false, /**< \f$ω⃗_ɑ\f$ (roll velocity)  */ 
        false, /**< \f$ω⃗_β\f$ (pitch velocity) */ 
        false  /**< \f$ω⃗_θ\f$ (yaw velocity)   */ 
    };

public: /* ------------------------------------------------ Node's parameters ----------------------------------------------------- */
    
    /// Description of the parameter defining acceleration gain of the robot's control vector
    static constexpr node_common::parameters::ParamDescriptor<std::vector<double>, robot_localization::TWIST_SIZE> CONTROL_ACC_GAIN_PARAM_DESCRIPTOR {
        .name           = "control_acc_gain",
        .read_only      = true,
        .dynamic_typing = false,
        .default_value  = std::array{ 1.3, 1.3, 1.3, 1.3, 1.3, 4.5 },
        .description    = "6-element vector representing acceleration gain of the robot's controls in the "
                          "{ v_x, v_y, v_z, v_roll, v_pitch, v_yaw } order"
    };

    /// Description of the parameter defining acceleration limits of the robot's control vector
    static constexpr node_common::parameters::ParamDescriptor<std::vector<double>, robot_localization::TWIST_SIZE> CONTROL_ACC_LIMITS_PARAM_DESCRIPTOR {
        .name           = "control_acc_limits",
        .read_only      = true,
        .dynamic_typing = false,
        .default_value  = std::array{ 0.8, 1.3, 1.3, 1.3, 1.3, 0.9 },
        .description    = "6-element vector representing acceleration limits of the robot's controls in the "
                          "{ v_x, v_y, v_z, v_roll, v_pitch, v_yaw } order"
    };

    /// Description of the parameter defining deceleration gain of the robot's control vector
    static constexpr node_common::parameters::ParamDescriptor<std::vector<double>, robot_localization::TWIST_SIZE> CONTROL_DEC_GAIN_PARAM_DESCRIPTOR {
        .name           = "control_dec_gain",
        .read_only      = true,
        .dynamic_typing = false,
        .default_value  = std::array{ 1.3, 1.3, 1.3, 1.3, 1.3, 4.5 },
        .description    = "6-element vector representing deceleration gain of the robot's controls in the "
                          "{ v_x, v_y, v_z, v_roll, v_pitch, v_yaw } order"
    };

    /// Description of the parameter defining deceleration limits of the robot's control vector
    static constexpr node_common::parameters::ParamDescriptor<std::vector<double>, robot_localization::TWIST_SIZE> CONTROL_DEC_LIMITS_PARAM_DESCRIPTOR {
        .name           = "control_dec_limits",
        .read_only      = true,
        .dynamic_typing = false,
        .default_value  = std::array{ 1.0, 1.3, 1.3, 1.3, 1.3, 1.0 },
        .description    = "6-element vector representing deceleration limits of the robot's controls in the "
                          "{ v_x, v_y, v_z, v_roll, v_pitch, v_yaw } order"
    };

    /// Description of the parameter defining timeout value, in seconds, after which a control is considered stale
    static constexpr node_common::parameters::ParamDescriptor<double> CONTROL_TIMEOUT_S_PARAM_DESCRIPTOR {
        .name           = "control_timeout_s",
        .read_only      = true,
        .dynamic_typing = false,
        .default_value  = 0.1,
        .description    = "Timeout value, in seconds, after which a control is considered stale"
    };
    
    /// Description of the parameter defining measurements Mahalanobis distance threshold in number of sigmas of the filter
    static constexpr node_common::parameters::ParamDescriptor<std::vector<double>, STATE_VECTOR_SIZE * STATE_VECTOR_SIZE>
    PROCESS_NOISE_COVARIANCE_MATRIX_PARAM_DESCRIPTOR {
        .name           = "process_noise_covariance",
        .read_only      = true,
        .dynamic_typing = false,
        .default_value  = std::array {

                0.05 , 0.0  , 0.0  , 0.0  , 0.0  , 0.0  , 0.0   , 0.0   , 0.0  , 0.0  , 0.0  , 0.0  , 0.0  , 0.0  , 0.0   ,
                0.0  , 0.05 , 0.0  , 0.0  , 0.0  , 0.0  , 0.0   , 0.0   , 0.0  , 0.0  , 0.0  , 0.0  , 0.0  , 0.0  , 0.0   ,
                0.0  , 0.0  , 0.06 , 0.0  , 0.0  , 0.0  , 0.0   , 0.0   , 0.0  , 0.0  , 0.0  , 0.0  , 0.0  , 0.0  , 0.0   ,
                0.0  , 0.0  , 0.0  , 0.03 , 0.0  , 0.0  , 0.0   , 0.0   , 0.0  , 0.0  , 0.0  , 0.0  , 0.0  , 0.0  , 0.0   ,
                0.0  , 0.0  , 0.0  , 0.0  , 0.03 , 0.0  , 0.0   , 0.0   , 0.0  , 0.0  , 0.0  , 0.0  , 0.0  , 0.0  , 0.0   ,
                0.0  , 0.0  , 0.0  , 0.0  , 0.0  , 0.06 , 0.0   , 0.0   , 0.0  , 0.0  , 0.0  , 0.0  , 0.0  , 0.0  , 0.0   ,
                0.0  , 0.0  , 0.0  , 0.0  , 0.0  , 0.0  , 0.025 , 0.0   , 0.0  , 0.0  , 0.0  , 0.0  , 0.0  , 0.0  , 0.0   ,
                0.0  , 0.0  , 0.0  , 0.0  , 0.0  , 0.0  , 0.0   , 0.025 , 0.0  , 0.0  , 0.0  , 0.0  , 0.0  , 0.0  , 0.0   ,
                0.0  , 0.0  , 0.0  , 0.0  , 0.0  , 0.0  , 0.0   , 0.0   , 0.04 , 0.0  , 0.0  , 0.0  , 0.0  , 0.0  , 0.0   ,
                0.0  , 0.0  , 0.0  , 0.0  , 0.0  , 0.0  , 0.0   , 0.0   , 0.0  , 0.01 , 0.0  , 0.0  , 0.0  , 0.0  , 0.0   ,
                0.0  , 0.0  , 0.0  , 0.0  , 0.0  , 0.0  , 0.0   , 0.0   , 0.0  , 0.0  , 0.01 , 0.0  , 0.0  , 0.0  , 0.0   ,
                0.0  , 0.0  , 0.0  , 0.0  , 0.0  , 0.0  , 0.0   , 0.0   , 0.0  , 0.0  , 0.0  , 0.02 , 0.0  , 0.0  , 0.0   ,
                0.0  , 0.0  , 0.0  , 0.0  , 0.0  , 0.0  , 0.0   , 0.0   , 0.0  , 0.0  , 0.0  , 0.0  , 0.01 , 0.0  , 0.0   ,
                0.0  , 0.0  , 0.0  , 0.0  , 0.0  , 0.0  , 0.0   , 0.0   , 0.0  , 0.0  , 0.0  , 0.0  , 0.0  , 0.01 , 0.0   ,
                0.0  , 0.0  , 0.0  , 0.0  , 0.0  , 0.0  , 0.0   , 0.0   , 0.0  , 0.0  , 0.0  , 0.0  , 0.0  , 0.0  , 0.015 ,
                
            },

        .description    = "Process (noise) uncertianty (covariance) 15x15 matrix of the model [Q] given as 225-element row-major array"
    };

    /// Description of the parameter defining timeout value, in seconds, after which a sensor measurement is considered stale
    static constexpr node_common::parameters::ParamDescriptor<double> SENSOR_TIMEOUT_S_PARAM_DESCRIPTOR {
        .name           = "sensor_timeout_s",
        .read_only      = true,
        .dynamic_typing = false,
        .default_value  = 0.1,
        .description    = "Timeout value, in seconds, after which a sensor measurement is considered stale"
    };

    /// Description of the parameter defining measurements Mahalanobis distance threshold in number of sigmas of the filter
    static constexpr node_common::parameters::ParamDescriptor<double> MAHALANOBIS_MEASUREMENT_THRESHOLD_PARAM_DESCRIPTOR {
        .name           = "mahalanobis_threshold",
        .read_only      = true,
        .dynamic_typing = false,
        .default_value  = 2,
        .description    = "The measurements Mahalanobis distance threshold in number of sigmas of the filter"
    };
    
    /// Description of the parameter defining name of the output file for the debug logs of the filter
    static constexpr node_common::parameters::ParamDescriptor<std::string> DEBUG_LOG_FILE_PARAM_DESCRIPTOR {
        .name           = "debug_log_file",
        .read_only      = true,
        .dynamic_typing = false,
        .description    = "Name of the output file for debug logs produced by the filtering module (set an empty "
                          "string to disable debug logging)"
    };

public: /* ------------------------------------------------ Topic's parameters ---------------------------------------------------- */
    
    /// Size of the topics' queue
    static constexpr std::size_t TOPIC_QUEUE_SIZE = 1000;

    /// Name of the topic subscribed by the node to acquire current velocity setpoint of the robot
    static constexpr auto VELOCITY_SETPOINT_SUB_TOPIC_NAME = "velocity_setpoint";
    /// Name of the topic subscribed by the node to acquire encoders-odometry-based position of the robot
    static constexpr auto ODOM_SUB_TOPIC_NAME = "odom/encoders";
    /// Name of the topic subscribed by the node to acquire laser-odometry-based position of the robot
    static constexpr auto LASER_ODOM_SUB_TOPIC_NAME = "odom/laser/pose";

    /// Name of the topic published by the node to broadcast filtered odometry data
    static constexpr auto ODOM_FILTERED_PUB_TOPIC_NAME = "odom/filtered";
    /// Name of the topic published by the node to broadcast filtered odometry pose
    static constexpr auto ODOM_FILTERED_POSE_PUB_TOPIC_NAME = "odom/filtered/pose";
    /// Name of the topic published by the node to broadcast filtered odometry twist
    static constexpr auto ODOM_FILTERED_TWIST_PUB_TOPIC_NAME = "odom/filtered/velocity";

    /// Name of the TF base frame for the output frame indicating filtered position
    static constexpr auto TF_BASE_FRAME = "odom";
    /// Name of the TF output frame indicating filtered position of the robot with respect to the @a TF_BASE_FRAME 
    static constexpr auto TF_OUT_FRAME = "odom_filtered_pose";

public: /* ----------------------------------------------- Public ctors & dtors --------------------------------------------------- */

    /**
     * @brief Construct a new OdomFusion node
     * 
     * @param options 
     *    configuration of the node
     */
    OdomFusion(const rclcpp::NodeOptions & options);

    /**
     * @brief Destroy the OdomFusion node
     */
    ~OdomFusion();

private: /* ------------------------------------------------ Callback methods ----------------------------------------------------- */

    /**
     * @brief Callback method called at arrival of the new velocity setpoint
     */
    void velocity_setpoint_callback(const geometry_msgs::msg::Twist &msg);

    /**
     * @brief Callback method called at arrival of the new encoder-based odometry data
     */
    void odom_callback(const nav_msgs::msg::Odometry &msg);

    /**
     * @brief Callback method called at arrival of the new laser-based odometry data
     */
    void laser_odom_callback(const geometry_msgs::msg::PoseStamped &msg);

private: /* ----------------------------------------------- Auxiliary methods ----------------------------------------------------- */

    /**
     * @brief Publishes current output of the filter to both output topic and TF channel
     * @param now 
     *    timestamp to be assigned to published messages
     */
    void publish(const rclcpp::Time &now);

private: /* ------------------------------------------------- ROS interfaces ------------------------------------------------------ */

	/// Subscriber interface used to acquire velocity setpoint for the robot
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_setpoint_sub;
	/// Subscriber interface used to acquire encoder-based odometry data
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
	/// Subscriber interface used to acquire laser-based odometry data
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr laser_odom_sub;

	/// Publisher interface used to broadcast filtered odometry data
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_filtered_pub;
	/// Publisher interface used to broadcast filtered odometry pose
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr odom_filtered_pose_pub;
	/// Publisher interface used to broadcast filtered odometry velocity
	rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr odom_filtered_twist_pub;

    /// TF2 publishing object for broadcastign output frames
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

    /// Message buffer for the filtered odometry data being published by the node (kept as member object to avoid header copypasting) 
    nav_msgs::msg::Odometry odom_msg { rosidl_runtime_cpp::MessageInitialization::ZERO };
    /// Message buffer for the TF transformation being published by the node (kept as member object to avoid header copypasting) 
    geometry_msgs::msg::TransformStamped tf_out_msg { rosidl_runtime_cpp::MessageInitialization::ZERO };

private: /* -------------------------------------------------- Node's state ------------------------------------------------------- */

    /// Handle to the debug log output for the filter
    std::optional<std::ofstream> debug_out_stream;
    /// Robot localization EK (Extended Kalman Filter) filter
    robot_localization::Ekf filter;
    /// The measurements Mahalanobis distance threshold in number of sigmas of the filter
    double mahalanobis_measurement_threshold;

    /// Last controls vector
    Eigen::VectorXd last_controls { Eigen::VectorXd::Zero(robot_localization::TWIST_SIZE) };
    /// Timestamp of the last controls
    rclcpp::Time last_controls_stamp { this->get_clock()->now() };

};

/* ================================================================================================================================ */

} // End namespace velmwheel

#endif
