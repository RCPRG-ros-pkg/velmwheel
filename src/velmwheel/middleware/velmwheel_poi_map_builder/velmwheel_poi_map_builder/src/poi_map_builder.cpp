/* ============================================================================================================================ *//**
 * @file       poi_map_builder.cpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 31st March 2022 9:53:56 pm
 * @modified   Thursday, 26th May 2022 2:40:06 am
 * @project    engineering-thesis
 * @brief      Definition of the ROS2 component node class building an intensity map of points of interest (POI)
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

/* =========================================================== Includes =========================================================== */

// System includes
#include <exception>
#include <utility>
// TF includes
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
// Common includes
#include "node_common/node.hpp"
// Private includes
#include "velmwheel/poi_map_builder.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace velmwheel::points_of_interest {

/* ======================================================= Helper functions ======================================================= */

/**
 * @brief Helper function converting distance matching threshold from vector to target structure
 */
static inline const std::array<double, MapBuilder::POSITION_2D_SIZE> 
vector_to_distance_matching_threshold(const std::vector<double> &vector) {

    std::array<double, MapBuilder::POSITION_2D_SIZE> ret;

    // Set the point's coordinates
    if(vector.size() >= MapBuilder::POSITION_2D_SIZE) {
        ret[0] = vector[0];
        ret[1] = vector[1];
    } else
        throw std::range_error{ "[MapBuilder] Cannot convert std::vector to distance-matching-threshold array (not enough elements)" };

    return ret;
};

/* ========================================================= Ctors & dtors ======================================================== */


MapBuilder::MapBuilder(const rclcpp::NodeOptions & options) : 
    rclcpp::Node(NODE_NAME, options)
{
    /* ------------------------ Register parameters ------------------------ */

    // Node's common parameters    
    auto distance_limit             = node_common::parameters::declare_parameter_and_get( *this, DISTANCE_LIMIT_PARAM_DESCRIPTOR             );
    auto intensity_threshold        = node_common::parameters::declare_parameter_and_get( *this, INTENSITY_THRESHOLD_PARAM_DESCRIPTOR        );
    auto fixed_frame                = node_common::parameters::declare_parameter_and_get( *this, FIXED_FRAME_PARAM_DESCRIPTOR                );
    auto robot_frame                = node_common::parameters::declare_parameter_and_get( *this, ROBOT_FRAME_PARAM_DESCRIPTOR                );
    auto intensity_limit            = node_common::parameters::declare_parameter_and_get( *this, INTENSITY_LIMIT_PARAM_DESCRIPTOR            );
    auto save_intensity_threshold   = node_common::parameters::declare_parameter_and_get( *this, SAVE_INTENSITY_THRESHOLD_PARAM_DESCRIPTOR   );
    auto visualization_threshold    = node_common::parameters::declare_parameter_and_get( *this, VISUALIZATION_THRESHOLD_PARAM_DESCRIPTOR    );
    auto visualization_publish_rate = node_common::parameters::declare_parameter_and_get( *this, VISUALIZATION_PUBLISH_RATE_PARAM_DESCRIPTOR );

    // Node's map parameters
    auto map_markers_limit               = node_common::parameters::declare_parameter_and_get( *this, MAP_MARKERS_LIMIT_PARAM_DESCRIPTOR               );
    auto map_markers_namespace           = node_common::parameters::declare_parameter_and_get( *this, MAP_MARKERS_NAMESPACE_PARAM_DESCRIPTOR           );
    auto map_distance_matching_threshold = node_common::parameters::declare_parameter_and_get( *this, MAP_DISTANCE_MATCHING_THRESHOLD_PARAM_DESCRIPTOR );

    // Node's temporary map parameters
    auto temporary_map_markers_limit               = node_common::parameters::declare_parameter_and_get( *this, TEMPORARY_MAP_MARKERS_LIMIT_PARAM_DESCRIPTOR               );
    auto temporary_map_markers_namespace           = node_common::parameters::declare_parameter_and_get( *this, TEMPORARY_MAP_MARKERS_NAMESPACE_PARAM_DESCRIPTOR           );
    auto temporary_map_distance_matching_threshold = node_common::parameters::declare_parameter_and_get( *this, TEMPORARY_MAP_DISTANCE_MATCHING_THRESHOLD_PARAM_DESCRIPTOR );

    /* ------------------------- Validate parameters ----------------------- */
        
    // Check if positive distance limit has been given
    if(distance_limit <= 0.0)
        rclcpp::exceptions::InvalidParametersException("'distance_limit' must be positive");
        
    // Check if non-empty reference frames has been given
    if(fixed_frame->empty())
        rclcpp::exceptions::InvalidParametersException("'fixed_frame' frame canot be an empty string");
    if(robot_frame->empty())
        rclcpp::exceptions::InvalidParametersException("'robot_frame' frame canot be an empty string");
        
    // Check if publication rate is not negative
    if(*visualization_publish_rate < 0)
        rclcpp::exceptions::InvalidParametersException("'visualization_publish_rate' rate cannot be negative");
    
    // Check if distance-matching threshold touples have a valid size
    if(map_distance_matching_threshold->size() != POSITION_2D_SIZE)
        rclcpp::exceptions::InvalidParametersException("'map_distance_matching_threshold' pose-like touple should be of size 2");
    if(temporary_map_distance_matching_threshold->size() != POSITION_2D_SIZE)
        rclcpp::exceptions::InvalidParametersException("'temporary_map_distance_matching_threshold' pose-like touple should be of size 2");
    
    /* -------------------- Register subscribe interfaces ------------------ */

    *node_common::communication::make_subscriber_builder(cloud_sub)
        .node(*this)
        .name(CLOUD_SUB_TOPIC_NAME)
        .qos(TOPIC_QUEUE_SIZE)
        .callback(*this, &MapBuilder::cloud_callback);

    /* --------------------- Register publish interfaces ------------------- */

    *node_common::communication::make_publisher_builder(markers_pub)
        .node(*this)
        .name(MARKERS_PUB_TOPIC_NAME)
        .qos(TOPIC_QUEUE_SIZE);

    *node_common::communication::make_publisher_builder(markers_map_pub)
        .node(*this)
        .name(MARKERS_MAP_PUB_TOPIC_NAME)
        .qos(TOPIC_QUEUE_SIZE);

    *node_common::communication::make_publisher_builder(visualization_pub)
        .node(*this)
        .name(VISUALIZATION_PUB_TOPIC_NAME)
        .qos(TOPIC_QUEUE_SIZE);
        
    /* -------------------- Register service interfaces -------------------- */

    *node_common::communication::make_service_builder(change_mode_srv)
        .node(*this)
        .name(CHANGE_MODE_SRV_TOPIC_NAME)
        .callback(*this, &MapBuilder::change_mode_callback);

    *node_common::communication::make_service_builder(save_map_srv)
        .node(*this)
        .name(SAVE_MAP_SRV_TOPIC_NAME)
        .callback(*this, &MapBuilder::save_map_callback);

    *node_common::communication::make_service_builder(load_map_srv)
        .node(*this)
        .name(LOAD_MAP_SRV_TOPIC_NAME)
        .callback(*this, &MapBuilder::load_map_callback);

    /* --------------------- Register cyclical routines -------------------- */
    
    if(*visualization_publish_rate > 0.0) {
        *node_common::node::make_wall_timer_builder(timer)
            .node(*this)
            .rate(*visualization_publish_rate)
            .callback(*this, &MapBuilder::publish_visualization);
    }
    
    /* ---------------------- Initialize TF interfaces --------------------- */
    
    // Prepare TF2 buffer for the TF2 transform-listener interface
    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    // Create TF2 transform-listener interface with the given buffer
    transform_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    
    /* --------------------- Initialize internal state --------------------- */

    // Construct the map builder
    map_builder.emplace(
        *MapBuilderImpl::make_constructor()
            .tf_buffer(*tf_buffer)
            .distance_limit(*distance_limit)
            .intensity_threshold(*intensity_threshold)
            .fixed_frame(*fixed_frame)
            .robot_frame(*robot_frame)
            .intensity_limit(*intensity_limit)
            .save_intensity_threshold(*save_intensity_threshold)
            .map_markers_limit(*map_markers_limit)
            .map_distance_matching_threshold_xy(vector_to_distance_matching_threshold(*map_distance_matching_threshold))
            .temporary_map_markers_limit(*temporary_map_markers_limit)
            .temporary_map_distance_matching_threshold_xy(vector_to_distance_matching_threshold(*temporary_map_distance_matching_threshold))
    );

    /// Construct map-markers visualizer
    map_markers_visualizer.emplace(
        *poi_map::MapVisualizer::make_constructor()
            .score_threshold(*visualization_threshold)
            .ns(RVIZ_MAP_MARKERS_NS)
            .id(RVIZ_MARKERS_ID)
            .duration(rclcpp::Duration::from_nanoseconds(0))
            .scale_xy(RVIZ_MARKERS_SCALE_XY)
            .color_rgba(RVIZ_MAP_MARKERS_COLOR_RGBA)
    );
    
    /// Construct momentary-map-markers visualizer
    temporary_map_markers_visualizer.emplace(
        *poi_map::MapVisualizer::make_constructor()
            .score_threshold(*visualization_threshold)
            .ns(RVIZ_TEMPORARY_MAP_MARKERS_NS)
            .id(RVIZ_MARKERS_ID)
            .duration(rclcpp::Duration::from_nanoseconds(0))
            .scale_xy(RVIZ_MARKERS_SCALE_XY)
            .color_rgba(RVIZ_TEMPORARY_MAP_MARKERS_COLOR_RGBA)
    );

    /* ------------------------------------------------------------------------------- */
    
    node_common::node::print_hello(*this);
}


MapBuilder::~MapBuilder() {
    node_common::node::print_goodbye(*this);
}

/* =========================================================== Callbacks ========================================================== */
        
void MapBuilder::cloud_callback(const sensor_msgs::msg::PointCloud2 &msg) {

    // Proceede if node is not in the Idle mdoe
    if(mode != Mode::Idle) {

        // Try to update map(s) with incoming scan
        try {
            if(mode == Mode::Mapping)
                map_builder->update_map(msg);
            else if(mode == Mode::Watching)
                map_builder->update_temporary_map(msg);
        } catch(std::exception &ex) {
            RCLCPP_WARN_STREAM(this->get_logger(), "Failed to update intensity map(s) based on incoming scan (" << ex.what() << ")");
            return;
        }

        poi_map_msgs::msg::MarkersStamped temporary_map_msg;

        // Fill header of the map to be published
        temporary_map_msg.header.stamp = msg.header.stamp;
        // Fill message's header
        temporary_map_msg.header.frame_id = map_builder->get_robot_frame();
        // Fill message's body
        temporary_map_msg.markers = map_builder->get_temporary_map(msg.header.stamp).in_frame(map_builder->get_robot_frame());
        // Publish the temproary map
        markers_pub->publish(temporary_map_msg);
        
        // If in map mode, publish map markers
        if(mode == Mode::Mapping)
            publish_map(msg.header.stamp);
            
    }
}


void MapBuilder::change_mode_callback(
    const velmwheel_poi_map_builder_msgs::srv::ChangeMode::Request::SharedPtr req,
    velmwheel_poi_map_builder_msgs::srv::ChangeMode::Response::SharedPtr res
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


void MapBuilder::save_map_callback(
    const poi_map_msgs::srv::SaveMap::Request::SharedPtr req,
    poi_map_msgs::srv::SaveMap::Response::SharedPtr res
) {
    // Save current map
    map_builder->save_map(req->file_path);

    // Mark request as sucesfully fulfilled
    res->succeeded = true;
}


void MapBuilder::load_map_callback(
    const poi_map_msgs::srv::LoadMap::Request::SharedPtr req,
    poi_map_msgs::srv::LoadMap::Response::SharedPtr res
) {

    // Load map from the file
    map_builder->load_map(req->file_path);

    // Publish updated map
    publish_map(this->get_clock()->now(), true);

    // Mark request as sucesfully fulfilled
    res->succeeded = true;
}

/* ======================================================= Auxiliary methods ====================================================== */

void MapBuilder::publish_map(
    const rclcpp::Time &time,
    bool map_reloaded
) {
    
    // Get name of the fixed frame
    auto &fframe = map_builder->get_fixed_frame();

    poi_map_msgs::msg::MarkersMap map_msg;

    // Fill message's header
    map_msg.header.stamp    = time;
    map_msg.header.frame_id = fframe;
    // Fill message's body
    map_msg.markers  = map_builder->get_map(time).in_frame(fframe);
    map_msg.reloaded = map_reloaded;

    // Publish the map
    markers_map_pub->publish(map_msg);
}


void MapBuilder::publish_visualization() {

    // Proceede if node is not in the Idle mdoe
    if(mode != Mode::Idle) {

        // Get name of the fixed frame
        auto &fframe = map_builder->get_fixed_frame();
        // Get name of the robot's frame
        auto &rframe = map_builder->get_robot_frame();

        // Get current time
        auto now = this->get_clock()->now();
        // Get current map
        auto map = map_builder->get_map(now).in_frame(fframe);
        // Get current temporary map (in robot's frame of reference)
        auto temporary_map = map_builder->get_temporary_map(now).in_frame(rframe);

        visualization_msgs::msg::MarkerArray msg;

        // Add visualizations to the message
        msg.markers.push_back(map_markers_visualizer           -> visualize( now, fframe, map           ));
        msg.markers.push_back(temporary_map_markers_visualizer -> visualize( now, rframe, temporary_map ));
        
        // Publish the message
        visualization_pub->publish(msg);

    }
}

/* ================================================================================================================================ */

} // End namespace velmwheel::points_of_interest

/* ======================================================== Nodes' registry ======================================================= */

RCLCPP_COMPONENTS_REGISTER_NODE(velmwheel::points_of_interest::MapBuilder)

/* ================================================================================================================================ */

