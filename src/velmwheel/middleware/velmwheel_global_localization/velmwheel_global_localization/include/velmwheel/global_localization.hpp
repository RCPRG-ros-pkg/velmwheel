/* ============================================================================================================================ *//**
 * @file       global_localization.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 31st March 2022 11:42:25 pm
 * @modified   Thursday, 26th May 2022 2:41:33 am
 * @project    engineering-thesis
 * @brief      Declarations of the ROS2 node class implementing global localization node for the WUT Velmwheel robot based on matching
 *             of characteristic points from the robot's environment
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __VELMWHEEL_GLOBAL_LOCALIZATION_H__
#define __VELMWHEEL_GLOBAL_LOCALIZATION_H__

/* =========================================================== Includes =========================================================== */

// System includes
#include <array>
#include <optional>
// ROS includes
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rclcpp_components/register_node_macro.hpp"
// Interfaces includes
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "poi_map_msgs/msg/markers_stamped.hpp"
#include "poi_map_msgs/msg/markers_map.hpp"
#include "poi_map_msgs/srv/save_map.hpp"
#include "poi_map_msgs/srv/load_map.hpp"
#include "velmwheel_global_localization_msgs/srv/change_mode.hpp"
// TF includes
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
// Common includes
#include "node_common/parameters.hpp"
#include "node_common/communication.hpp"
// Private includes
#include "velmwheel/global_localization_impl.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace velmwheel {

/* ============================================================= Node ============================================================= */

/**
 * @brief ROS2 node class implementing global localization node for the WUT Velmwheel 
 *    robot based on matching of characteristic points from the robot's environment
 */
class RCLCPP_PUBLIC GlobalLocalization : public rclcpp::Node {

public: /* -------------------------------------------------- Node's traits ------------------------------------------------------- */

    /// Name of the node
    static constexpr auto NODE_NAME = "global_localization";

public: /* ------------------------------------------------ Node's parameters ----------------------------------------------------- */

    /// Description of the parameter defining default STD points-matching methods
    static constexpr node_common::parameters::ParamDescriptor<std::string> STD_MATCHING_METHOD_PARAM_DESCRIPTOR {
        .name                   = "std_matching_method",
        .read_only              = true,
        .dynamic_typing         = false,
        .default_value          = "previous_transform",
        .description            = "Default method of markers-matching (STD localization)",
        .additional_constraints = "One of { 'previous_transform', 'neighbour_distance', 'neighbour_translation' }"
    };

    /// Description of the parameter defining default STD points-matching methods
    static constexpr node_common::parameters::ParamDescriptor<std::string> STD_MATCHING_METHOD_FALLBACK_PARAM_DESCRIPTOR {
        .name                   = "std_matching_method_fallback",
        .read_only              = true,
        .dynamic_typing         = false,
        .default_value          = "neighbour_distance",
        .description            = "Fall-back method of markers-matching (STD localization)",
        .additional_constraints = "One of { 'previous_transform', 'neighbour_distance', 'neighbour_translation' }"
    };

    /// List of valid points-matching methods
    static constexpr auto VALID_MATCHIN_METHODS = std::array{ "previous_transform", "neighbour_distance", "neighbour_translation" };

    /// Description of the parameter defining markers-matching distance limit
    static constexpr node_common::parameters::ParamDescriptor<double> STD_PREVIOUS_TRANSFORM_MATCH_THRESHOLD_PARAM_DESCRIPTOR {
        .name           = "std_previous_transform_match_threshold",
        .read_only      = true,
        .dynamic_typing = false,
        .default_value  = 0.25,
        .description    = "Maximal distance between estimated position of the incoming marker in the 'map' frame of reference "
                          "and position of the compared map marker for which node assumes that both points correspond to each "
                          "other in [m] used when the 'previous_transform' markers-matching method is used (STD localization)"
    };

    /// Description of the parameter defining 
    static constexpr node_common::parameters::ParamDescriptor<double> STD_MEAN_NEIGHBOURS_DISTANCE_THRESHOLD_PARAM_DESCRIPTOR {
        .name           = "std_mean_neighbours_distance_threshold",
        .read_only      = true,
        .dynamic_typing = false,
        .default_value  = 0.15,
        .description    = "Maximal value of the mean difference betwen point-to-neighbour distances of matched "
                          "neighbours in the 'neighbour_distance' points-matching mode in [m] (STD localization)"
    };

    /// Description of the parameter defining 
    static constexpr node_common::parameters::ParamDescriptor<double> STD_MEAN_NEIGHBOURS_TRANSLATION_THRESHOLD_PARAM_DESCRIPTOR {
        .name           = "std_mean_neighbours_translation_threshold",
        .read_only      = true,
        .dynamic_typing = false,
        .default_value  = 0.15,
        .description    = "Maximal value of the mean distance betwen point-to-neighbour translations of matched "
                          "neighbours in the 'neighbour_translation' points-matching mode in [m] (STD localization)"
    };
    
    /// Description of the parameter defining maximum correspondance distance for accepted match pairs
    static constexpr node_common::parameters::ParamDescriptor<double> ICP_MAX_CORRESPONDANCE_DISTANCE_PARAM_DESCRIPTOR {
        .name           = "icp_max_correspondance_distance",
        .read_only      = true,
        .dynamic_typing = false,
        .default_value  = 100.0,
        .description    = "Maximum correspondance distance for accepted match pairs (ICP localization)"
    };

    /// Description of the parameter defining epsilon (maximal accepted inter-iteration change) in the ICP transformation
    static constexpr node_common::parameters::ParamDescriptor<double> ICP_TRANSFORMATION_EPSILON_PARAM_DESCRIPTOR {
        .name           = "icp_transformation_epsilon",
        .read_only      = true,
        .dynamic_typing = false,
        .default_value  = 1e-10,
        .description    = "Epsilon (maximal accepted inter-iteration change) in the ICP transformation (ICP localization)"
    };

    /// Description of the parameter defining epsilon (maximal accepted inter-iteration change) in the fitness of point clouds 
    static constexpr node_common::parameters::ParamDescriptor<double> ICP_EUCLIDEAN_FITNESS_EPSILON_PARAM_DESCRIPTOR {
        .name           = "icp_euclidean_fitness_epsilon",
        .read_only      = true,
        .dynamic_typing = false,
        .default_value  = 0.001,
        .description    = "Epsilon (maximal accepted inter-iteration change) in the fitness of point clouds (ICP localization)"
    };

    /// Description of the parameter defining maximum number of tierations of the ICP algorithm
    static constexpr node_common::parameters::ParamDescriptor<int> ICP_MAXIMUM_ITERATIONS_PARAM_DESCRIPTOR {
        .name           = "icp_maximum_iterations",
        .read_only      = true,
        .dynamic_typing = false,
        .default_value  = 100,
        .description    = "Maximum number of tierations of the ICP algorithm (ICP localization)"
    };

public: /* ------------------------------------------------ Topic's parameters ---------------------------------------------------- */
    
    /// Size of the topics' queue
    static constexpr std::size_t TOPIC_QUEUE_SIZE = 50;

	/// Name of the subscriber interfaces for incoming odometry data
	static constexpr auto ODOM_SUB_TOPIC_NAME = "odom";
	/// Name of the subscriber interface for incoming markers parsed from environment scans
	static constexpr auto MARKERS_SUB_TOPIC_NAME = "markers";
	/// Name of the subscriber interface for incoming markers from the built intensity map
	static constexpr auto MARKERS_MAP_SUB_TOPIC_NAME = "markers_map";

	/// Name of the publisher interface used to provide output pose stamped of the robot
	static constexpr auto POSE_PUB_TOPIC_NAME = "pose";

	/// Name of the service interface used to change node's mode
	static constexpr auto CHANGE_MODE_SRV_TOPIC_NAME = "change_mode";

public: /* --------------------------------------------------- TF parameters ------------------------------------------------------ */

    /// Name of the TF reference frame of the published estimated pose tranfromation
	static constexpr auto TF_REFERENCE_FRAME_NAME = "map";
	/// Name of the TF target frame of the published estimated pose tranfromation
	static constexpr auto TF_TARGET_FRAME_NAME = "odom";

public: /* ----------------------------------------------- Public ctors & dtors --------------------------------------------------- */

    /**
     * @brief Construct a new GlobalLocalization node
     * 
     * @param options 
     *    configuration of the node
     */
    GlobalLocalization(const rclcpp::NodeOptions &options);

    /**
     * @brief Destroy the GlobalLocalization object with a goodbye message
     */
    ~GlobalLocalization();

private: /* ------------------------------------------------ Callback methods ----------------------------------------------------- */
    
    /**
     * @brief Callback for incoming odometry data
     */
    void odom_callback(const nav_msgs::msg::Odometry &msg);
    
    /**
     * @brief Callback for the incoming intensity markers
     * @param msg 
     *    incoming intensity markers
     */
    void markers_callback(const poi_map_msgs::msg::MarkersStamped &msg);
    
    /**
     * @brief Callback for incoming markes of intensity map
     */
    void markers_map_callback(const poi_map_msgs::msg::MarkersMap &msg);

    /**
     * Callback for the change mode service request
     */
    void change_mode_callback(
        const velmwheel_global_localization_msgs::srv::ChangeMode::Request::SharedPtr req,
        velmwheel_global_localization_msgs::srv::ChangeMode::Response::SharedPtr res
    );

private: /* ------------------------------------------------- ROS interfaces ------------------------------------------------------ */

	/// Subscriber interfaces for incoming odometry data
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
	/// Publisher interface for incoming markers parsed from environment scans
	rclcpp::Subscription<poi_map_msgs::msg::MarkersStamped>::SharedPtr markers_sub;
	/// Publisher interface for incoming markers from the built intensity map
	rclcpp::Subscription<poi_map_msgs::msg::MarkersMap>::SharedPtr markers_map_sub;

	/// Publisher interface used to provide output pose of the robot
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;

	/// Service interface used to change node's mode
	rclcpp::Service<velmwheel_global_localization_msgs::srv::ChangeMode>::SharedPtr change_mode_srv;
    
    /// TF2 publishing object for broadcastign output frames
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

private: /* -------------------------------------------------- Node's state ------------------------------------------------------- */

    /**
     * @enum Mode
     * @brief Enumeration describing node's mode of opration
     */
    enum class Mode : unsigned {
        Idle            = velmwheel_global_localization_msgs::srv::ChangeMode::Request::IDLE,
        LocalizationSTD = velmwheel_global_localization_msgs::srv::ChangeMode::Request::LOCALIZATION_STD,
        LocalizationICP = velmwheel_global_localization_msgs::srv::ChangeMode::Request::LOCALIZATION_ICP
    };

    /// List of valid modes in a numerical form
    static constexpr auto VALID_MODES = std::array { 
        static_cast<unsigned>( Mode::Idle            ),
        static_cast<unsigned>( Mode::LocalizationSTD ),
        static_cast<unsigned>( Mode::LocalizationICP )
    };

    /// Current mode of operation
    Mode mode { Mode::Idle };

    /**
     * @note @a global_localizer is wrapped in std::optional to postopone construction
     *    until ROS porameters are loaded
     */

    /// Object responsible for actual localization
    std::optional<GlobalLocalizationImpl> global_localizer;

};

/* ================================================================================================================================ */

} // End namespace velmwheel

#endif
