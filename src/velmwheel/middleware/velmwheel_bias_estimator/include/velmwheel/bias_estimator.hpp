/* ============================================================================================================================ *//**
 * @file       bias_estimator.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 16th March 2022 4:37:20 pm
 * @modified   Thursday, 26th May 2022 2:33:00 am
 * @project    engineering-thesis
 * @brief      Declaration of the ROS2 node class implementing bias estimation algorithm for measurements from the IMU sensor
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __VELMWHEEL_BIAS_ESTIMATOR_H__
#define __VELMWHEEL_BIAS_ESTIMATOR_H__

/* =========================================================== Includes =========================================================== */

// System includes
#include <limits>
#include <utility>
// Math includes
#include "tf2_eigen/tf2_eigen.h"
// ROS includes
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rclcpp_components/register_node_macro.hpp"
// Standard messages
#include "std_msgs/msg/float64.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
// Private includes
#include "node_common/parameters.hpp"
#include "node_common/communication.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace velmwheel {

/* ============================================================= Node ============================================================= */

/**
 * @brief ROS2 node class estimation orientation and angular velocity of the robot based
 *    measurements from the IMU sensor and LIDAR-based odometry
 * @details The node implements a MKF (Multidimensional Kalman Filter) estimating current
 *    orientation and angular velocity of the robot based based measurements from the IMU
 *    sensor and LIDAR-based odometry. The state of the system is given as:
 *    
 *    \f[
 *        x[n] = \begin{bmatrix}
 *                     Θ     \\
 *                  ω_{bias}
 *               \end{bmatrix}
 *               \begin{matrix}
 *                  \textrm{estimated state of the system} \\
 *                  \textrm{}
 *               \end{matrix}
 *    \f]
 *   
 *          Θ - current orientation of the robot
 *     ω_bias - current bias of the measurement of angular velocity of the robot 
 *              provided by the IMU sensor
 * 
 *    The filtering model is implemented as follows:
 * 
 *      1) Prediction (Extrapolation) phase
 * 
 *        - system state prediction:
 *          
 *          \f[
 *              \begin{array}{c}
 *                  (x̂[n + 1|n]) \\
 *                  \begin{bmatrix}
 *                         Θ[n + 1|n]     \\
 *                      ω_{bias}[n + 1|n] 
 *                  \end{bmatrix} = 
 *              \end{array}
 *              \begin{array}{c}
 *                  (F) \\
 *                  \begin{bmatrix}
 *                      1 & -Δt \\
 *                      0 &  1  
 *                  \end{bmatrix} \times
 *              \end{array}
 *              \begin{array}{c}
 *                  (x̂[n|n]) \\
 *                  \begin{bmatrix}
 *                         Θ[n|n]    \\
 *                      ω_{bias}[n|n] 
 *                  \end{bmatrix} +
 *              \end{array}
 *              \begin{array}{c}
 *                  (G) \\
 *                  \begin{bmatrix}
 *                      0 & Δt \\
 *                      0 & 0  
 *                  \end{bmatrix} \times
 *              \end{array}
 *              \begin{array}{c}
 *                  (u) \\
 *                  \begin{bmatrix}
 *                      Θ_{odom} \\
 *                       ω_{imu} 
 *                  \end{bmatrix}
 *              \end{array}
 *          \f]
 * 
 *         - estimated uncertainty prediction:
 * 
 *             \f[
 *                 P[n + 1|n] = F \times P[n|n] * F^{T} + Q
 *             \f]
 *          
 *      2) Update (correction) phase
 * 
 *         - Kalman Gain update:
 *         
 *           \f[
 *              K[n] = P[n|n-1] \times H \times ( H \times P[n|n-1] \times H^{T} + R) \\
 *              H = \begin{bmatrix}
 *                      1 & 0
 *                  \end{bmatrix}
 *           \f]
 *         
 *         - System state estimation update
 *         
 *           \f[
 *               \begin{array}{c}
 *                   (x̂[n|n]) \\
 *                   \begin{bmatrix}
 *                          Θ[n|n]     \\
 *                       ω_{bias}[n|n] 
 *                   \end{bmatrix} = 
 *               \end{array}
 *               \begin{array}{c}
 *                   (x̂[n|n-1]) \\
 *                   \begin{bmatrix}
 *                          Θ[n|n-1]    \\
 *                       ω_{bias}[n|n-1] 
 *                   \end{bmatrix} +
 *               \end{array}
 *               \begin{array}{c}
 *                   (T) \\
 *                   \begin{bmatrix}
 *                       1 &   0 \\
 *                       0 & 1/Δt 
 *                   \end{bmatrix} \times
 *               \end{array}
 *               \begin{array}{c}
 *                   \\
 *                   K[N] \times ( z[n] - H \times x̂[n|n-1])
 *               \end{array}
 *           \f]
 *         
 *         - estimated uncertainty update (constant covariance dynamic assumed):
 *         
 *           \f[
 *               P[n|n] = P[n|n-1]
 *           \f]
 * 
 *    where
 *    
 *       * \f$Θ[n|n]\f$         - current estimation of the robot's orientation
 *       * \f$ω_{bias}[n|n]\f$  - current estimation of the velocity measurement's bias
 *       * \f$Δt\f$             - time elapsed since the last update/prediction
 *       * \f$Θ_{odom}\f$       - odometry estimation of robot's orientation
 *       * \f$ω_{imu}\f$        - angular velocity of the robot measured by the IMU sensors
 *       * \f$F\f$              - system transition matrix
 *       * \f$G\f$              - control matrix
 *       * \f$P[n|n-1]\f$       - estimate uncertainty matrix predicted in the last iteration
 *       * \f$H\f$              - observation matrix for the system
 *       * \f$R\f$              - measurement uncertainty matrix (scalar in this case)
 *       * \f$z[n]\f$           - current measurement} (\f$z[n] = [ Θ_{odom} ] := H \times x̂[n]\f$)
 *       * \f$x̂[n|n-1]\f$       - prediction of the system state from the previous iteraion
 *       * \f$z[n]\f$           - current measurement
 *       * \f$T\f$              - custom transformation matrix thats aim to estimate innovation of the \f$ω_{bias}\f$ from the innovation of the Θ
 * 
 */
class RCLCPP_PUBLIC BiasEstimator : public rclcpp::Node {

public: /* -------------------------------------------------- Node's traits ------------------------------------------------------- */

    /// Name of the node
    static constexpr auto NODE_NAME = "bias_estimator";

public: /* ------------------------------------------------ Node's parameters ----------------------------------------------------- */
    
    /// Number of elements in the vector initializing P matrix
    static constexpr std::size_t P_INI_SIZE = 4;
    /// Number of elements in the vector initializing Q matrix
    static constexpr std::size_t Q_INI_SIZE = 2;

    /// Description of the parameter defining initial value of the P matrix
    static constexpr node_common::parameters::ParamDescriptor<std::vector<double>, P_INI_SIZE> P_INIT_PARAM_DESCRIPTOR {
        .name           = "P_init",
        .read_only      = true,
        .dynamic_typing = false,
        .default_value  = std::array{ 0.0, 0.0, 0.0, 0.0 },
        .description    = "Initial value of the P matrix (estimate uncertainty) given as row-major 2x2 matrix"
    };
    
    /// Description of the parameter defining initial value of the Q matrix
    static constexpr node_common::parameters::ParamDescriptor<std::vector<double>, Q_INI_SIZE> Q_PARAM_DESCRIPTOR {
        .name           = "Q",
        .read_only      = true,
        .dynamic_typing = false,
        .default_value  = std::array{ 0.0, 0.0 },
        .description    = "Value of the diagonal elements of the Q matrix (process noise uncertainty)"
    };
    
    /// Description of the parameter defining initial value of the R matrix
    static constexpr node_common::parameters::ParamDescriptor<double> R_PARAM_DESCRIPTOR {
        .name           = "R",
        .read_only      = true,
        .dynamic_typing = false,
        .default_value  = 0.001,
        .description    = "Value of the measurement uncertainty of the filter (due to limited observation "
                          "matrix H it influences only odometry measurements)"
    };

public: /* ------------------------------------------------ Topic's parameters ---------------------------------------------------- */
    
    // Size of the topics' queue
    static constexpr std::size_t TOPIC_QUEUE_SIZE = 1000;

    /// Name of the topic subscribed by the node to acquire raw IMU measurements
    static constexpr auto IMU_SUB_TOPIC_NAME = "imu/out";
    /// Name of the topic subscribed by the node to acquire laser-odometry-based position of the robot
    static constexpr auto LASER_ODOM_SUB_TOPIC_NAME = "odom/laser/pose";

    /// Name of the topic published by the node to broadcast filtered IMU measurements
    static constexpr auto IMU_FILTERED_PUB_TOPIC_NAME = "imu/filtered";
    /// Name of the topic published by the node to broadcast current estimation of the theta angle
    static constexpr auto THETA_PUB_TOPIC_NAME = "orientation/filtered";
    
    /// Name of the service topic used to initialize processing of the algorithm
    static constexpr auto INITIALIZE_SRV_TOPIC_NAME = "initialize";

public: /* ----------------------------------------------- Public ctors & dtors --------------------------------------------------- */

    /**
     * @brief Construct a new BiasEstimator node
     * 
     * @param options 
     *    configuration of the node
     */
    BiasEstimator(const rclcpp::NodeOptions & options);

    /**
     * @brief Destructs BiasEstimator node
     */
    ~BiasEstimator();

private: /* ------------------------------------------------ Callback methods ----------------------------------------------------- */

    /**
     * @brief Callback method called at arrival of the new IMU measurement
     */
    void imu_callback(const sensor_msgs::msg::Imu &msg);

    /**
     * @brief Callback method called at arrival of the new laser-odometry-based pose estimation
     */
    void laser_odom_callback(const geometry_msgs::msg::PoseStamped &msg);

    /**
     * @brief Callback method to the service triggering start of the node's processing loop
     */
    void initialize_callback(
        const std_srvs::srv::Trigger::Request::SharedPtr req,
        std_srvs::srv::Trigger::Response::SharedPtr res
    );

private: /* ----------------------------------------------- Auxiliary methods ----------------------------------------------------- */

    /**
     * @param last_stamp 
     *    the last timestamp at which the method was called 
     * @returns 
     *    pair of values consisting of
     *        @retval current timepoint
     *        @retval time elapsed between @p last_stamp and current moment
     */
    inline std::pair<rclcpp::Time, rclcpp::Duration> get_current_time_diff(const rclcpp::Time &last_stamp);

    /**
     * @param time_diff 
     *    time interval of the between system's transitions
     * @returns 
     *    system's state transition matrix (designated as F or A) for the given time interval
     */
    inline Eigen::Matrix<double, 2, 2> get_state_transition_matrix(const rclcpp::Duration &time_diff);

    /**
     * @brief Publishes current estimation of the robot's orientation
     */
    inline void publish_orientation_estimation();

private: /* ------------------------------------------------- ROS interfaces ------------------------------------------------------ */

	/// Subscriber interface used to acquire raw IMU measurements
	rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
	/// Subscriber interface used to acquire laser-odometry-based position of the robot
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr laser_odom_sub;

	/// Publisher interface used to broadcast filtered IMU measurements
	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
	/// Publisher interface used to broadcast current estimation of the theta angle
	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr theta_pub;

    /// Service initializing processing the algorithm
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr initialize_srv;

private: /* -------------------------------------------------- Node's state ------------------------------------------------------- */

    /// Boolean flag starting processing loop of the filter
    bool initialized { false };

    /// P matrix accumulated by the filter (estimate uncertainty)
    Eigen::Matrix<double, 2, 2> P_matrix;
    /// Q diagonal matrix accumulated by the filter (process noise uncertainty)
    Eigen::Matrix<double, 2, 2> Q_matrix;
    /// K vector (Kalman gain)
    Eigen::Vector2d K_vector { 0.0, 0.0 };
    /// Measurement uncertainty of the filter (due to limited observation matrix H it influences only odometry measurements)
    double R;

    /// Current estimation of the robot's orientation in the Z axis in [rad]
    double robot_orientation_rad { 0.0 };
    /// Current estimation of the IMU bias of the robot's angular velocity in [rad/s]
    double bias_rad_s { 0.0 };

    /// Timestamp of the last IMU callback (initialized with negative value to indicate state before the first callback)
    rclcpp::Time last_imu_stamp { std::numeric_limits<int64_t>::min() };
    /// Timestamp of the last odometry callback (initialized with negative value to indicate state before the first callback)
    rclcpp::Time last_odom_stamp { std::numeric_limits<int64_t>::min() };

};

/* ================================================================================================================================ */

} // End namespace velmwheel

#endif
