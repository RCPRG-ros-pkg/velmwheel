/* ============================================================================================================================ *//**
 * @file       base_controller.cpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 16th March 2022 4:37:53 pm
 * @modified   Thursday, 26th May 2022 2:25:07 am
 * @project    engineering-thesis
 * @brief      Definition of methods of the ROS2-based class implementing controll node responsible for basic conrol over the 
 *             Velmwheel's driveline
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

/* =========================================================== Includes =========================================================== */

// Common includes
#include "node_common/communication.hpp"
#include "node_common/node.hpp"
// Private includes
#include "velmwheel/base_controller.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace velmwheel {

/* ========================================================= Ctors & dtors ======================================================== */

BaseController::BaseController(const rclcpp::NodeOptions & options) : 
    rclcpp::Node(NODE_NAME, options)
{
    /* ---------------------------- Initialize subscribers --------------------------- */

    // Initialize subscriber of the topic broadcasting encoders measurements
    *node_common::communication::make_subscriber_builder(encoders_sub)
        .node(*this)
        .name(ENCODERS_SUB_TOPIC_NAME)
        .qos(TOPIC_QUEUE_SIZE)
        .callback(*this, &BaseController::encoders_measurement_callback);

    // Initialize subscriber of the topic broadcasting velocity setpoints for the robot
    *node_common::communication::make_subscriber_builder(velocity_setpoint_sub)
        .node(*this)
        .name(VELOCITY_SETPOINT_SUB_TOPIC_NAME)
        .qos(TOPIC_QUEUE_SIZE)
        .callback(*this, &BaseController::velocity_setpoint_callback);
    
    /* ----------------------------- Initialize publishers --------------------------- */

    // Initialize publisher for the topic broadcasting controls for robot's wheels
    *node_common::communication::make_publisher_builder(controls_pub)
        .node(*this)
        .name(CONTROLS_PUB_TOPIC_NAME)
        .qos(TOPIC_QUEUE_SIZE);
        
    // Initialize publisher for the topic broadcasting robot's velocity
    *node_common::communication::make_publisher_builder(velocity_pub)
        .node(*this)
        .name(VELOCITY_PUB_TOPIC_NAME)
        .qos(TOPIC_QUEUE_SIZE);
        
    // Initialize publisher for the topic broadcasting robot's wheels-based odometry
    *node_common::communication::make_publisher_builder(odom_pub)
        .node(*this)
        .name(ODOM_PUB_TOPIC_NAME)
        .qos(TOPIC_QUEUE_SIZE);
        
    // Initialize publisher for the topic broadcasting robot's wheels-based odometry as a pose
    *node_common::communication::make_publisher_builder(odom_pose_pub)
        .node(*this)
        .name(ODOM_POSE_PUB_TOPIC_NAME)
        .qos(TOPIC_QUEUE_SIZE);

    // Initialize TF2 broadcaster
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    /* ------------------------------ Initialize odometry ---------------------------- */

    // Fill message's header
    odom_msg.header.frame_id = ODOM_FRAME;
    odom_msg.child_frame_id  = ODOM_CHILD_FRAME;
    // Initialize odometry covariance
    odom_msg.pose.covariance  = DEFAULT_ODOM_COVARIANCE;
    odom_msg.twist.covariance = DEFAULT_ODOM_COVARIANCE;

    // Initialize odom transformation
    odom_transform.header.frame_id = ODOM_FRAME;
    odom_transform.child_frame_id  = ODOM_CHILD_FRAME;
 
    /* ------------------------------------------------------------------------------- */
    
    node_common::node::print_hello(*this);

}


BaseController::~BaseController() {
    node_common::node::print_goodbye(*this);   
}

/* =========================================================== Callbacks ========================================================== */

void BaseController::encoders_measurement_callback(const velmwheel_msgs::msg::EncodersStamped &msg) {

    // Get current time
    auto now = this->get_clock()->now();

    /* ---------------------------- Publish robot's velocity ------------------------- */

    velmwheel_msgs::msg::Wheels wheel_velocities;

    // Parse wheel velocities
    for(unsigned i = 0; i < velmwheel_msgs::msg::WheelEnum::NUM; ++i)
        wheel_velocities.values[i] = msg.encoders[i].velocity;

    geometry_msgs::msg::TwistStamped velocity_msg;

    // Fill message's header
    velocity_msg.header.stamp    = now;
    velocity_msg.header.frame_id = velmwheel::params::ROBOT_NAME;
    // Fill message's body
    velocity_msg.twist = velmwheel::math::wheels_to_twist(wheel_velocities);
    // Public the message
    velocity_pub->publish(velocity_msg);

    /* ----------------------------- Publish odometry data --------------------------- */

    velmwheel_msgs::msg::Wheels wheel_angles;

    // Parse wheel velocities
    for(unsigned i = 0; i < velmwheel_msgs::msg::WheelEnum::NUM; ++i)
        wheel_angles.values[i] = msg.encoders[i].angle;
    
    // Fill message's header
    odom_msg.header.stamp = now;
    // Fill message's body
    odom_msg.twist.twist = velocity_msg.twist;
    odom_msg.pose        = update_odom(wheel_angles);
    // Public the message
    odom_pub->publish(odom_msg);

    /* ----------------------------- Publish odometry pose --------------------------- */

    geometry_msgs::msg::PoseStamped pose_msg;

    // Fill message's header
    pose_msg.header.stamp    = now;
    pose_msg.header.frame_id = ODOM_FRAME;
    // Fill message's body
    pose_msg.pose = odom_msg.pose.pose;
    // Public the message
    odom_pose_pub->publish(pose_msg);

    /* ------------------- Publish odom -> base_link transformation ------------------ */

    // Fill message's header
    odom_transform.header.stamp = now;
    // Fill message's body
    odom_transform.transform.translation.x = odom_msg.pose.pose.position.x;
    odom_transform.transform.translation.y = odom_msg.pose.pose.position.y;
    odom_transform.transform.translation.z = 0.0;
    odom_transform.transform.rotation      = odom_msg.pose.pose.orientation;
    // Public the message
    tf_broadcaster->sendTransform(odom_transform);

}


void BaseController::velocity_setpoint_callback(const geometry_msgs::msg::Twist &msg) {
    controls_pub->publish(velmwheel::math::twist_to_wheels(msg));
}

/* ============================================================ Helpers =========================================================== */

geometry_msgs::msg::PoseWithCovariance BaseController::update_odom(const velmwheel_msgs::msg::Wheels &current_angles) {    
    
    /* ------------------------- Calculate current odometry -------------------------- */

    velmwheel_msgs::msg::Wheels angles_diff{ rosidl_runtime_cpp::MessageInitialization::SKIP };

    // Calculate differences in angles
    for(unsigned i = 0; i < velmwheel_msgs::msg::WheelEnum::NUM; ++i)
        angles_diff.values[i] = current_angles.values[i] - odom_keepup.previous_angles.values[i];
    // Estimated rotation and translation since the last iteration 
    geometry_msgs::msg::Twist diff = velmwheel::math::wheels_to_twist(angles_diff);

    /**
     * @note The formula used to calculate translation and rotation since the last iteration
     *    is the same one that related velocities of robot's wheels with it's linear and angular
     *    velocity. This is justified as long as we assume that the change in the encoder's
     *    angles is linear between controll iterations. This exactly the assumption that the
     *    simple, linearized-integration-based odometry does.
     */

    geometry_msgs::msg::PoseWithCovariance pose;

    // Calculate current Z angle
    double z_angle = odom_keepup.previous_z_angle + diff.angular.z;
    // Transform displacement into the vector
    tf2::Vector3 displacement = tf2::Vector3{ diff.linear.x, diff.linear.y, diff.linear.z };
    // Transform displacement to the global frame of reference by rotating it by robot's orientation quaternion
    displacement = displacement.rotate(tf2::Vector3{0, 0, 1}, z_angle);

    // Compute current orientation
    pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3{0, 0, 1}, z_angle));
    // Compute current position
    pose.pose.position.x = odom_keepup.previous_position.x + displacement.x();
    pose.pose.position.y = odom_keepup.previous_position.y + displacement.y();
    pose.pose.position.z = 0;
    
    /* --------------- Keep odom informations for the next iteration ----------------- */

    // Update odometry info
    odom_keepup.previous_position = pose.pose.position;
    odom_keepup.previous_z_angle  = z_angle;
    odom_keepup.previous_angles   = current_angles;

    /* ------------------------------------------------------------------------------- */
    
    return pose;
}

/* ================================================================================================================================ */

} // End namespace velmwheel

/* ======================================================== Nodes' registry ======================================================= */

RCLCPP_COMPONENTS_REGISTER_NODE(velmwheel::BaseController)

/* ================================================================================================================================ */

