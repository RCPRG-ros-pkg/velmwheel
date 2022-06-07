/* ============================================================================================================================ *//**
 * @file       global_localization.cpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 16th March 2022 4:37:53 pm
 * @modified   Thursday, 26th May 2022 2:41:58 am
 * @project    engineering-thesis
 * @brief      Definitions of the ROS2 node class implementing routines of the global localization node for the WUT Velmwheel robot based
 *             on matching of characteristic points from the robot's environment
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

/* =========================================================== Includes =========================================================== */

// Common includes
#include "node_common/node.hpp"
// TF includes
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
// Private includes
#include "velmwheel/global_localization.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace velmwheel {

/* ========================================================= Ctors & dtors ======================================================== */

GlobalLocalization::GlobalLocalization(const rclcpp::NodeOptions &options) : 
    rclcpp::Node(NODE_NAME, options)
{
    /* ----------------------------- Declare parameters ------------------------------ */

    auto std_matching_method                       = node_common::parameters::declare_parameter_and_get( *this, STD_MATCHING_METHOD_PARAM_DESCRIPTOR                       );
    auto std_matching_method_fallback              = node_common::parameters::declare_parameter_and_get( *this, STD_MATCHING_METHOD_FALLBACK_PARAM_DESCRIPTOR              );
    auto std_previous_transform_match_threshold    = node_common::parameters::declare_parameter_and_get( *this, STD_PREVIOUS_TRANSFORM_MATCH_THRESHOLD_PARAM_DESCRIPTOR    );
    auto std_mean_neighbours_distance_threshold    = node_common::parameters::declare_parameter_and_get( *this, STD_MEAN_NEIGHBOURS_DISTANCE_THRESHOLD_PARAM_DESCRIPTOR    );
    auto std_mean_neighbours_translation_threshold = node_common::parameters::declare_parameter_and_get( *this, STD_MEAN_NEIGHBOURS_TRANSLATION_THRESHOLD_PARAM_DESCRIPTOR );
    auto icp_max_correspondance_distance           = node_common::parameters::declare_parameter_and_get( *this, ICP_MAX_CORRESPONDANCE_DISTANCE_PARAM_DESCRIPTOR           );
    auto icp_transformation_epsilon                = node_common::parameters::declare_parameter_and_get( *this, ICP_TRANSFORMATION_EPSILON_PARAM_DESCRIPTOR                );
    auto icp_euclidean_fitness_epsilon             = node_common::parameters::declare_parameter_and_get( *this, ICP_EUCLIDEAN_FITNESS_EPSILON_PARAM_DESCRIPTOR             );
    auto icp_maximum_iterations                    = node_common::parameters::declare_parameter_and_get( *this, ICP_MAXIMUM_ITERATIONS_PARAM_DESCRIPTOR                    );

    /* ------------------------------- Verify parameters ----------------------------- */

    // Check whether valid points-matching methods has been given
    if(std::find(VALID_MATCHIN_METHODS.begin(), VALID_MATCHIN_METHODS.end(), *std_matching_method) == VALID_MATCHIN_METHODS.end())
        rclcpp::exceptions::InvalidParametersException("'std_matching_method' is invalid");
    if(std::find(VALID_MATCHIN_METHODS.begin(), VALID_MATCHIN_METHODS.end(), *std_matching_method_fallback) == VALID_MATCHIN_METHODS.end())
        rclcpp::exceptions::InvalidParametersException("'std_matching_method_fallback' is invalid");

    // Check whether positive number of iterations for ICP algorithm has been given
    if(*icp_maximum_iterations <= 0)
        rclcpp::exceptions::InvalidParametersException("'icp_maximum_iterations' must be positive");

    /* ---------------------------- Initialize subscribers --------------------------- */

    *node_common::communication::make_subscriber_builder(odom_sub)
        .node(*this)
        .name(ODOM_SUB_TOPIC_NAME)
        .qos(TOPIC_QUEUE_SIZE)
        .callback(*this, &GlobalLocalization::odom_callback);

    *node_common::communication::make_subscriber_builder(markers_sub)
        .node(*this)
        .name(MARKERS_SUB_TOPIC_NAME)
        .qos(TOPIC_QUEUE_SIZE)
        .callback(*this, &GlobalLocalization::markers_callback);

    *node_common::communication::make_subscriber_builder(markers_map_sub)
        .node(*this)
        .name(MARKERS_MAP_SUB_TOPIC_NAME)
        .qos(TOPIC_QUEUE_SIZE)
        .callback(*this, &GlobalLocalization::markers_map_callback);

    /* ----------------------------- Initialize publishers --------------------------- */

    *node_common::communication::make_publisher_builder(pose_pub)
        .node(*this)
        .name(POSE_PUB_TOPIC_NAME)
        .qos(TOPIC_QUEUE_SIZE);

    /* ---------------------------- Initialize subscribers --------------------------- */

    *node_common::communication::make_service_builder(change_mode_srv)
        .node(*this)
        .name(CHANGE_MODE_SRV_TOPIC_NAME)
        .callback(*this, &GlobalLocalization::change_mode_callback);

    /* --------------------------- Initialize TF interfaces -------------------------- */

    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    /* ------------------- Initialize Global Localization subsystem ------------------ */

    // Helper functor converting string representation of the points-matching method to enum
    auto points_matching_method_from_string = [](const std::string &str) {
        if(str == "previous_transform")
            return GlobalLocalizationImpl::ConfigSTD::MatchingMethod::PreviousTransform;
        else if(str == "neighbour_distance")
            return GlobalLocalizationImpl::ConfigSTD::MatchingMethod::NeighboursDistance;
        else if(str == "neighbour_translation")
            return GlobalLocalizationImpl::ConfigSTD::MatchingMethod::NeighboursTranslation;
        throw std::runtime_error{ "Invalid points-matching method has been set and not detected by the node in params-validation phase" };
    };

    // Initialize global-localization subsystem
    global_localizer.emplace(
        GlobalLocalizationImpl::ConfigSTD {
            .matching_method                         = points_matching_method_from_string(*std_matching_method),
            .fallback_matching_method                = points_matching_method_from_string(*std_matching_method_fallback),
            .previous_transform_match_threshold      = *std_previous_transform_match_threshold,
            .mean_neighbours_distance_threshold_m    = *std_mean_neighbours_distance_threshold,
            .mean_neighbours_translation_threshold_m = *std_mean_neighbours_translation_threshold
        },
        GlobalLocalizationImpl::ConfigICP {
            .max_correspondance_distance = *icp_max_correspondance_distance,
            .transformation_epsilon      = *icp_transformation_epsilon,
            .euclidean_fitness_epsilon   = *icp_euclidean_fitness_epsilon,
            .maximum_iterations          = static_cast<unsigned>(*icp_maximum_iterations)
        }
    );

    /* ------------------------------------------------------------------------------- */
    
    node_common::node::print_hello(*this);
}


GlobalLocalization::~GlobalLocalization() {
    node_common::node::print_goodbye(*this);
}

/* =========================================================== Callbacks ========================================================== */


void GlobalLocalization::odom_callback(const nav_msgs::msg::Odometry &msg) {

    // If non-idle mode is set, update odometry
    if(mode != Mode::Idle)
        global_localizer->update_odometry(msg);
        
}


void GlobalLocalization::markers_callback(const poi_map_msgs::msg::MarkersStamped &msg) {
    
    // If non-idle mode is set, update scan data
    if(mode != Mode::Idle) {

        // Update incoming laser scan in the localizer
        global_localizer->update_scan(msg);

        geometry_msgs::msg::Pose odom_pose_estimation;

        // Update current estimation
        std::optional<tf2::Transform> odom_to_map_transform_estimation;
        switch(mode) {
            case Mode::LocalizationSTD: odom_to_map_transform_estimation = global_localizer->estimate_odom_pose_std(); break;
            case Mode::LocalizationICP: odom_to_map_transform_estimation = global_localizer->estimate_odom_pose_icp(); break;
            default:
                break;
        }
        // Check whether estimation succeeded
        if(not odom_to_map_transform_estimation.has_value()) {
            RCLCPP_WARN_STREAM(this->get_logger(), "Failed to estimate current correction of the 'odom' frame");
            return;
        }

        /* --------------------- Publish current ROS message --------------------- */

        geometry_msgs::msg::PoseStamped pose_msg;

        // Convert TF transformation to msg
        auto odom_to_map_msg_estimation = tf2::toMsg(*odom_to_map_transform_estimation);
        // Fill message's header
        pose_msg.header.frame_id = TF_TARGET_FRAME_NAME;
        pose_msg.header.stamp    = msg.header.stamp;
        // Fill message's body
        pose_msg.pose.position.x  = odom_to_map_msg_estimation.translation.x;
        pose_msg.pose.position.y  = odom_to_map_msg_estimation.translation.y;
        pose_msg.pose.position.z  = odom_to_map_msg_estimation.translation.z;
        pose_msg.pose.orientation = odom_to_map_msg_estimation.rotation;
        // Publish the message
        pose_pub->publish(pose_msg);

        /* ------------------ Publish current TF transformation ------------------ */

        geometry_msgs::msg::TransformStamped pose_transform;

        // Fill message's header
        pose_transform.header.frame_id = TF_REFERENCE_FRAME_NAME;
        pose_transform.header.stamp    = msg.header.stamp;
        pose_transform.child_frame_id  = TF_TARGET_FRAME_NAME;
        // Fill message's body
        pose_transform.transform = odom_to_map_msg_estimation;
        // Publish the message
        pose_pub->publish(pose_msg);

    }

}


void GlobalLocalization::markers_map_callback(const poi_map_msgs::msg::MarkersMap &msg) {
    
    // If non-idle mode is set, update map
    if(mode != Mode::Idle)
        global_localizer->update_map(msg);

}


void GlobalLocalization::change_mode_callback(
    const velmwheel_global_localization_msgs::srv::ChangeMode::Request::SharedPtr req,
    velmwheel_global_localization_msgs::srv::ChangeMode::Response::SharedPtr res
) {
    
    // Check if vali mode has been given
    if(VALID_MODES.end() == std::find(VALID_MODES.begin(), VALID_MODES.end(), req->mode)) {

        // Mark request as failed
        res->succeeded = false;
        // Pass human-readible description
        res->reason = "Invalid mode has been requested";

    }

    // Change the mode
    mode = static_cast<Mode>(req->mode);
    // Mark request as successfull
    res->succeeded = true;
    
}

/* ================================================================================================================================ */

} // End namespace velmwheel

/* ======================================================== Nodes' registry ======================================================= */

RCLCPP_COMPONENTS_REGISTER_NODE(velmwheel::GlobalLocalization)

/* ================================================================================================================================ */

