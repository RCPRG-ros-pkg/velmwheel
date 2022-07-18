/* ============================================================================================================================ *//**
 * @file       bias_estimator.cpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 16th March 2022 4:37:53 pm
 * @modified   Monday, 18th July 2022 7:24:50 pm
 * @project    engineering-thesis
 * @brief      Definition of the ROS2 node class implementing bias estimation algorithm for measurements from the IMU sensor
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

/* =========================================================== Includes =========================================================== */

// Common includes
#include "node_common/node.hpp"
// TF includes
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"
// Interfaces includes
#include "std_msgs/msg/float64.hpp"
// Private includes
#include "velmwheel/bias_estimator.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace velmwheel {

/* ========================================================= Ctors & dtors ======================================================== */

BiasEstimator::BiasEstimator(const rclcpp::NodeOptions & options) : 
    rclcpp::Node(NODE_NAME, options)
{
    /* ----------------------------- Initialize parameters --------------------------- */

    // Declare parameter defining initial value of the P matrix
    auto p_init = node_common::parameters::declare_parameter_and_get(*this, P_INIT_PARAM_DESCRIPTOR);
    // Declare parameter defining initial value of the Q matrix
    auto q_init = node_common::parameters::declare_parameter_and_get(*this, Q_PARAM_DESCRIPTOR);
    // Declare parameter defining initial value of the R matrix
    auto r_init = node_common::parameters::declare_parameter_and_get(*this, R_PARAM_DESCRIPTOR);
    
    /* ----------------------------- Validate parameters ----------------------------- */

    // Check if a valid P has been given
    if(not p_init.has_value() or p_init->size() != 2 * 2)
        rclcpp::exceptions::InvalidParametersException("Initial value of the P matrix should be given as an array with 2x2 elements");

    // Check if a valid Q has been given
    if(not q_init.has_value() or q_init->size() != 2 * 2)
        rclcpp::exceptions::InvalidParametersException("Initial value of the Q matrix should be given as an array with 2 diagonal elements");

    /* ---------------------------- Initialize subscribers --------------------------- */

    // Initialize IMU input subscriber
    *node_common::communication::make_subscriber_builder(imu_sub)
        .node(*this)
        .name(IMU_SUB_TOPIC_NAME)
        .qos(TOPIC_QUEUE_SIZE)
        .callback(*this, &BiasEstimator::imu_callback);
    
    // Initialize odom input subscriber
    *node_common::communication::make_subscriber_builder(laser_odom_sub)
        .node(*this)
        .name(LASER_ODOM_SUB_TOPIC_NAME)
        .qos(TOPIC_QUEUE_SIZE)
        .callback(*this, &BiasEstimator::laser_odom_callback);
    
    /* ----------------------------- Initialize publishers --------------------------- */
            
    // Initialize filtered IMU publisher
    *node_common::communication::make_publisher_builder(imu_pub)
        .node(*this)
        .name(IMU_FILTERED_PUB_TOPIC_NAME)
        .qos(TOPIC_QUEUE_SIZE);
        
    // Initialize theta estimation publisher
    *node_common::communication::make_publisher_builder(theta_pub)
        .node(*this)
        .name(THETA_PUB_TOPIC_NAME)
        .qos(TOPIC_QUEUE_SIZE);
    
    /* ------------------------------ Initialize services ---------------------------- */

    // Initialize service used to initialize the processing loop
    *node_common::communication::make_service_builder(initialize_srv)
        .node(*this)
        .name(INITIALIZE_SRV_TOPIC_NAME)
        .callback(*this, &BiasEstimator::initialize_callback);
    
    /* ---------------------------- Initialize private data -------------------------- */

    // Parse initial values for P matrix
    P_matrix << p_init->at(0), p_init->at(1),
                p_init->at(2), p_init->at(3);
    // Parse values for Q matrix
    Q_matrix << q_init->at(0),           0.0,
                          0.0, q_init->at(1);
    // Parse values for R matrix
    R = *r_init;

    /* ------------------------------------------------------------------------------- */

    RCLCPP_INFO( this->get_logger(), "Running bias estimator for IMU sensor with following parameters: " );
    RCLCPP_INFO( this->get_logger(), "                                                                 " );
    RCLCPP_INFO( this->get_logger(), "  P_init = [ %.4f %.4f ]", P_matrix(0, 0), P_matrix(1, 0)          );
    RCLCPP_INFO( this->get_logger(), "           [ %.4f %.4f ]", P_matrix(0, 1), P_matrix(1, 1)          );
    RCLCPP_INFO( this->get_logger(), "       Q = [ %.4f %.4f ]", Q_matrix(0, 0), Q_matrix(1, 0)          );
    RCLCPP_INFO( this->get_logger(), "           [ %.4f %.4f ]", Q_matrix(0, 1), Q_matrix(1, 1)          );
    RCLCPP_INFO( this->get_logger(), "                                                                 " );
    RCLCPP_INFO( this->get_logger(), "       R = %.4f", R                                                );
    RCLCPP_INFO( this->get_logger(), "                                                                 " );

    /* ------------------------------------------------------------------------------- */
    
    node_common::node::print_hello(*this);
}


BiasEstimator::~BiasEstimator() {
    node_common::node::print_goodbye(*this); 
}

/* =========================================================== Callbacks ========================================================== */

void BiasEstimator::imu_callback(const sensor_msgs::msg::Imu &msg) {

    // If node has not been initialized, return
    if(not initialized)
        return;

    // Get current time and duration since the last message
    auto [now, time_diff] = get_current_time_diff(last_imu_stamp);

    /* ---------------------------- Perform filtering ---------------------------- */

    // Get current system transition matrix based on the time interval
    Eigen::Matrix<double, 2, 2> F_matrix = get_state_transition_matrix(time_diff);

    /**
     * Calculate current \f$x̂[n + 1|n]\f$ prediction of the system state from the following extrapolation model
     * 
     *     \f[
     *         \begin{array}{c}
     *             (x̂[n + 1|n]) \\
     *             \begin{bmatrix}
     *                    Θ[n + 1|n]     \\
     *                 ω_{bias}[n + 1|n] 
     *             \end{bmatrix} = 
     *         \end{array}
     *         \begin{array}{c}
     *             (F) \\
     *             \begin{bmatrix}
     *                 1 & -Δt \\
     *                 0 &  1  
     *             \end{bmatrix} \times
     *         \end{array}
     *         \begin{array}{c}
     *             (x̂[n|n]) \\
     *             \begin{bmatrix}
     *                    Θ[n|n]    \\
     *                 ω_{bias}[n|n] 
     *             \end{bmatrix} +
     *         \end{array}
     *         \begin{array}{c}
     *             (G) \\
     *             \begin{bmatrix}
     *                 0 & Δt \\
     *                 0 & 0  
     *             \end{bmatrix} \times
     *         \end{array}
     *         \begin{array}{c}
     *             (u) \\
     *             \begin{bmatrix}
     *                 Θ_{odom} \\
     *                  ω_{imu} 
     *             \end{bmatrix}
     *         \end{array}
     *     \f]
     * 
     * where given are
     * 
     *    * \f$Θ[n|n]\f$      - current estimation of the robot's orientation
     *    * \f$ω_bias[n|n]\f$ - current estimation of the velocity measurement's bias
     *    * \f$Δt\f$          - time elapsed since the last IMU measurement
     *    * \f$Θ_odom\f$      - odometry estimation of robot's orientation
     *    * \f$ω_imu\f$       - angular velocity of the robot measured by the IMU sensors
     *    
     * and
     * 
     *    * F - system transition matrix
     *    * G - control matrix
     * 
     * @note Θ_odom measurement is not used in the prediction step and so the equation for Θ prediction
     *    is skipped
     */
    robot_orientation_rad = robot_orientation_rad + time_diff.seconds() * (msg.angular_velocity.z - bias_rad_s);

    /**
     * Predict estimated uncertainty matrix P[n + 1|n] with the following formula:
     * 
     *    \f[
     *        P[n + 1|n] = F \times P[n|n] * F^{T} + Q
     *    \f]
     * 
     * @todo Ask Wojtek Dudek why the inversion of the T matrix and no transposition
     *    is used
     */
    P_matrix = F_matrix * P_matrix * F_matrix.inverse() + Q_matrix;

    /* ----------------------- Publish current estimations ----------------------- */

    sensor_msgs::msg::Imu msg_filtered = msg;

    // Update header of the output message
    msg_filtered.header.stamp = now;
    // Update body of the output message
    msg_filtered.angular_velocity.z -= bias_rad_s;

    // Publish the filtered IMU measurement
    imu_pub->publish(msg_filtered);
    // Publish current estimation of the robot's orientation
    publish_orientation_estimation();

    /* --------------------------------------------------------------------------- */

    // Update callback's timestamp for use in the future handle routine
    last_imu_stamp = now;
}


void BiasEstimator::laser_odom_callback(const geometry_msgs::msg::PoseStamped &msg) {

    // If node has not been initialized, return
    if(not initialized)
        return;

    // Get current time and duration since the last message
    auto [now, time_diff] = get_current_time_diff(last_odom_stamp);

    tf2::Quaternion q;

    // Convert message to the tf2::Quaternion
    tf2::fromMsg(msg.pose.orientation, q);
    // Get current orientation of the robot estimated with laser-based odometry
    auto robot_odom_orientation_rad = tf2::getYaw(q);
    // If the filter has just been initialized, make current estimation of the robot's orientation equal to the odometry-based estimation
    if(not last_odom_stamp.has_value())
        robot_orientation_rad = robot_odom_orientation_rad;

    /* ---------------------------- Perform filtering ---------------------------- */
    
    /**
     * Update Kalman Gain K[n] of the system as:
     * 
     *       \f[
     *          K[n] = P[n|n-1] \times H \times ( H \times P[n|n-1] \times H^{T} + R)
     *       \f]
     * 
     * where
     * 
     *    * P[n|n-1] - estimate uncertainty matrix predicted in the last iteration
     *    * H        - observation matrix for the system
     *    * R        - measurement uncertainty matrix (scalar in this case)
     * 
     * @note For \f$H = [1 0]\f$ observation matrix the update equation for Kalman Gain
     *    can be rewritten as
     *    
     *    \f[
     *        K[n] = \frac{\begin{bmatrix} P[n|n-1](0, 0) \\ P[n|n-1](0, 1) \end{bmatrix}}{P[n|n-1](0, 0) + R}
     *    \f]
     */

    // Calculate denominator of the Kalman Gain
    double S = P_matrix(0,0) + R;
    // Calculate Kalman gain
    K_vector(0) = P_matrix(0, 0) / S;
    K_vector(1) = P_matrix(1, 0) / S;

    /**
     * Update current estimation of the system state using the inovation expression (difference
     * between actual and anticipated measurement). Assuming observations model:
     * 
     *   \f[
     *       z[n] = \begin{bmatrix} 
     *                  Θ_odom
     *              \end{bmatrix} = H \times 
     *              \begin{bmatrix}
     *                     Θ[n]   \\
     *                  ω_{bias}[n]
     *              \end{bmatrix}, 
     *              H = \begin{bmatrix}1 0\end{bmatrix}
     *   \f]
     * 
     * Calculate:
     * 
     *   \f[
     *       \begin{array}{c}
     *           (x̂[n|n]) \\
     *           \begin{bmatrix}
     *                  Θ[n|n]     \\
     *               ω_{bias}[n|n] 
     *           \end{bmatrix} = 
     *       \end{array}
     *       \begin{array}{c}
     *           (x̂[n|n-1]) \\
     *           \begin{bmatrix}
     *                  Θ[n|n-1]    \\
     *               ω_{bias}[n|n-1] 
     *           \end{bmatrix} +
     *       \end{array}
     *       \begin{array}{c}
     *           (T) \\
     *           \begin{bmatrix}
     *               1 &   0 \\
     *               0 & 1/Δt 
     *           \end{bmatrix} \times
     *       \end{array}
     *       \begin{array}{c}
     *           \\
     *           K[N] \times ( z[n] - H \times x̂[n|n-1])
     *       \end{array}
     *   \f]   
     * 
     * where
     * 
     *    * \f$x̂[n|n-1]\f$ - prediction of the system state from the previous iteraion
     *    * \f$z[n]\f$     - current measurement
     *    * \f$T\f$        - custom transformation matrix thats aim to estimate innovation of the \f$ω_{bias}\f$
     *                       from the innovation of the Θ
     *  
     * @note T matrix is a non-standard extension for the Kalman filter. It's aim is to compensate
     *    the fact that the direct measurement of the robot's velocity given by the IMU sensor
     *    is not used in the 'Update' step of the filter. In this step the filter's input is limited
     *    to the odometry information
     * 
     * @todo Ask Wojtek Dudek why such an estimation has been used instead of using direct measurement
     *    from the IMU
     */

    // Calculate innovation of the filter (residuum)
    double bias_correction = robot_odom_orientation_rad - robot_orientation_rad;

    // Update current estimation of the robot's orientation based on the innovation
    robot_orientation_rad = robot_orientation_rad + K_vector(0) * bias_correction;
    // Update current estimation of the bias based on the innovation
    if(time_diff.seconds() != 0)
        bias_rad_s = bias_rad_s + K_vector(1) * bias_correction / time_diff.seconds();

    /**
     * Update estimated uncertainty matrix P[|n] with the following formula:
     * 
     *     P[n|n] = P[n|n-1] (constant covariance dynamic assumed)
     * 
     * @todo Ask Wojtek Dudek why the estimated uncertainty update is skipped instead
     *   of being calculated as
     * 
     *   \f[
     *       P[n|n] = (I - K[n] \times H) \times P[n|n-1] \times (I - K[n]H)^{T} + K[n] \times R \times K[n]^{T}
     *   \f]
     *              
     */

    /* ----------------------- Publish current estimations ----------------------- */
    
    // Publish current estimation of the robot's orientation
    publish_orientation_estimation();

    /* --------------------------------------------------------------------------- */
    
    // Update callback's timestamp for use in the future handle routine
    last_odom_stamp = now;
}


void BiasEstimator::initialize_callback(
    [[maybe_unused]] const std_srvs::srv::Trigger::Request::SharedPtr req,
    std_srvs::srv::Trigger::Response::SharedPtr res
) {
    // If filter already initialized
    if(initialized) {
        
        // Fill the response
        res->success = false;
        res->message = "Filter already initialized";

    // Otherwise
    } else {

        // Fill the response
        res->success = true;
        res->message = "Filter initialized succesfully";
        // Mark filter as initialized
        initialized = true;
        
    }
}

/* ======================================================== Helper methods ======================================================== */

std::pair<rclcpp::Time, rclcpp::Duration> BiasEstimator::get_current_time_diff(const std::optional<rclcpp::Time> &last_stamp) {

    // Get current system time
    auto now = this->get_clock()->now();
    // Initialize time difference
    auto time_diff = rclcpp::Duration::from_nanoseconds(0);
    // If the callback is called any but the first time, calculate actual time difference
    if(last_stamp.has_value())
        time_diff = now - (*last_stamp);

    return std::make_pair(now, time_diff);
}


Eigen::Matrix<double, 2, 2> BiasEstimator::get_state_transition_matrix(const rclcpp::Duration &time_diff) {

    Eigen::Matrix<double, 2, 2> F_matrix;

    /**
     * Fill the state transition matrix for the system:
     * 
     *    \f[
     *        F = \begin{bmatrix}
     *              1 & -Δt \\
     *              0 &  1
     *            \end{bmatrix}
     *    \f]
     * 
     */
    F_matrix << 1.0, - time_diff.seconds(),
                0.0,                   1.0;

    return F_matrix;
}


void BiasEstimator::publish_orientation_estimation() {

    std_msgs::msg::Float64 msg;

    // Prepare message
    msg.data = robot_orientation_rad;

    // Publish message
    theta_pub->publish(msg);
}

/* ================================================================================================================================ */

} // End namespace velmwheel

/* ======================================================== Nodes' registry ======================================================= */

RCLCPP_COMPONENTS_REGISTER_NODE(velmwheel::BiasEstimator)

/* ================================================================================================================================ */

