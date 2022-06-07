/* ============================================================================================================================ *//**
 * @file       poi_map_builder.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 31st March 2022 9:16:44 pm
 * @modified   Thursday, 26th May 2022 12:06:29 am
 * @project    engineering-thesis
 * @brief      Declaration of the ROS2 component node class building an intensity map of points of interest (POI)
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __VELMWHEEL_POI_MAP_BUILDER_H__
#define __VELMWHEEL_POI_MAP_BUILDER_H__

/* =========================================================== Includes =========================================================== */

// ROS includes
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rclcpp_components/register_node_macro.hpp"
// Interfaces includes
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "poi_map_msgs/msg/markers_stamped.hpp"
#include "poi_map_msgs/msg/markers_map.hpp"
#include "poi_map_msgs/srv/save_map.hpp"
#include "poi_map_msgs/srv/load_map.hpp"
#include "velmwheel_poi_map_builder_msgs/srv/change_mode.hpp"
// TF includes
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
// Common includes
#include "node_common.hpp"
// Private includes
#include "velmwheel/params.hpp"
#include "velmwheel/poi_map_builder_impl.hpp"
#include "poi_map/map_visualizer.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace velmwheel::points_of_interest {

/* ============================================================= Node ============================================================= */

/**
 * @brief ROS2 component node class building an intensity map of points of interest (POI)
 * @details MapBuilder class focuses on building a map of points of interest based
 *    on incoming scans of the robot's environment. Such a map can be used by downstream
 *    nodes e.g. to provide global localization mechanism in some non-evolving setting.
 *    At the moment, the node is adapted to taken environment-scan-data in form of 
 *    sensor_msgs::msg::PointCloud2 clouds containing basic XYZ coordinates of scanned points
 *    along with their intensities which is the cases in the WUT Velmwheel robot system
 *    that processes scans from LIDAR sensors in this form.
 * 
 *    From the birdseye perspective, the node accumulates two sets of intensity markers 
 *    called 'map' and 'temporary' map. The first represent contemporary set of characteristic
 *    poitns that have been found in the environment. The secend refers to 'candidate' points
 *    found in the current close sorrounding of the robot that the global 'map' will updated
 *    based on. For more details see description of the @ref MapBuilderImpl class.
 * 
 *    The node provides a mechanism to periodically publish RVIZ-compliant 'Marker' messages
 *    visualizing current intensity map as well as the temporary markers map. It cano operate
 *    in on of three modes (acluding 'Idle' mode):
 * 
 *       Mapping ) In this mode incoming scan data are used to update both 'temporary map' and 
 *                 'global map'. Each map is publish to the output topic after being updated.
 * 
 *       Watching ) In this mode incoming scan data are used to update only 'temporary map'. 
 *                  It is publish to the output topic after being updated. 
 *     
 * @note The 'temporary map' markers are output in the robot's frame of reference while 'global map'
 *    markers are output in the map's frame of reference
 */
class RCLCPP_PUBLIC MapBuilder: public rclcpp::Node {

public: /* -------------------------------------------------- Node's traits ------------------------------------------------------- */

    /// Name of the node
    static constexpr auto NODE_NAME = "poi_map_builder";

public: /* --------------------------------------------- Node's common parameters ------------------------------------------------- */
    
    /// Size of the RGBA touple
    static constexpr std::size_t RGBA_TOUPLE_SIZE = 4;
    /// Size of the 2D position vector
    static constexpr std::size_t POSITION_2D_SIZE = 2;

    /// Description of the parameter defining maximal distance between subsequent points to be processed
    static constexpr node_common::parameters::ParamDescriptor<double> DISTANCE_LIMIT_PARAM_DESCRIPTOR {
        .name           = "distance_limit",
        .read_only      = true,
        .dynamic_typing = false,
        .default_value  = 0.05,
        .description    = "Maximal distance between subsequent points to be processed in [m]"
    };

    /// Description of the parameter defining intensity of the laser-scanned point to be considered as a boundary of the ROI
    static constexpr node_common::parameters::ParamDescriptor<int> INTENSITY_THRESHOLD_PARAM_DESCRIPTOR {
        .name           = "intensity_threshold",
        .read_only      = true,
        .dynamic_typing = false,
        .default_value  = 1500,
        .description    = "Threshold intensity of the laser-scanned point to be considered as a boundary "
                          "of the ROI (region of intereset)"
    };
    
    /// Description of the parameter defining name of the fixed frame of reference for markers in the resulting map
    static constexpr node_common::parameters::ParamDescriptor<std::string> FIXED_FRAME_PARAM_DESCRIPTOR {
        .name           = "fixed_frame",
        .read_only      = true,
        .dynamic_typing = false,
        .default_value  = "map",
        .description    = "Name of the fixed frame of reference for markers in the resulting map"
    };

    /// Description of the parameter defining name of the robot's local TF frame
    static constexpr node_common::parameters::ParamDescriptor<std::string> ROBOT_FRAME_PARAM_DESCRIPTOR {
        .name           = "robot_frame",
        .read_only      = true,
        .dynamic_typing = false,
        .default_value  = velmwheel::params::ROBOT_NAME,
        .description    = "Name of the robot's local TF frame"
    };
    
    /// Description of the parameter defining intensity limit of the map's markers
    static constexpr node_common::parameters::ParamDescriptor<double> INTENSITY_LIMIT_PARAM_DESCRIPTOR {
        .name           = "intensity_limit",
        .read_only      = true,
        .dynamic_typing = false,
        .default_value  = 100'000.0,
        .description    = "Intensity limit of the map's markers"
    };

    /// Description of the parameter defining threshold value determining minimal intensity of the marker to not being discarded when storing map into XML file
    static constexpr node_common::parameters::ParamDescriptor<double> SAVE_INTENSITY_THRESHOLD_PARAM_DESCRIPTOR {
        .name           = "save_intensity_threshold",
        .read_only      = true,
        .dynamic_typing = false,
        .default_value  = 200.0,
        .description    = "Threshold value determining minimal intensity of the marker to not being discarded when storing map into XML file"
    };

    /// Description of the parameter defining minimal intensity of incoming markers being visualized
    static constexpr node_common::parameters::ParamDescriptor<double> VISUALIZATION_THRESHOLD_PARAM_DESCRIPTOR {
        .name           = "visualization_threshold",
        .read_only      = true,
        .dynamic_typing = false,
        .default_value  = 1'000.0,
        .description    = "Minimal intensity of markers being visualized"
    };
    
    /// Description of the parameter defining frequency of publication of the visualization
    static constexpr node_common::parameters::ParamDescriptor<double> VISUALIZATION_PUBLISH_RATE_PARAM_DESCRIPTOR {
        .name           = "visualization_publish_rate",
        .read_only      = true,
        .dynamic_typing = false,
        .default_value  = 10.0,
        .description    = "Frequency of publication of the visualization in [Hz] (if 0, visualization is disabled)"
    };
    
public: /* ---------------------------------------------- Node's map parameters --------------------------------------------------- */

    /// Description of the parameter defining maximal number of markers in the map
    static constexpr node_common::parameters::ParamDescriptor<int> MAP_MARKERS_LIMIT_PARAM_DESCRIPTOR {
        .name           = "map_markers_limit",
        .read_only      = true,
        .dynamic_typing = false,
        .default_value  = 200,
        .description    = "Maximal number of markers in the map"
    };

    /// Description of the parameter defining name of the RVIZ markers namespace associated with markers present in the map
    static constexpr node_common::parameters::ParamDescriptor<std::string> MAP_MARKERS_NAMESPACE_PARAM_DESCRIPTOR {
        .name           = "map_markers_namespace",
        .read_only      = true,
        .dynamic_typing = false,
        .default_value  = "map_markers",
        .description    = "Name of the RVIZ markers namespace associated with markers present in the map"
    };
    
    /// Description of the parameter defining distance threshold for matching markers with map based on their position
    static constexpr node_common::parameters::ParamDescriptor<std::vector<double>, POSITION_2D_SIZE> MAP_DISTANCE_MATCHING_THRESHOLD_PARAM_DESCRIPTOR {
        .name           = "map_distance_matching_threshold",
        .read_only      = true,
        .dynamic_typing = false,
        .default_value  = std::array{ 0.4, 0.1 },
        .description    = "Array containing { dx, dy } coordinates defining maximal distance between temporary-map markers "
                          "and map markes to be considered matching"
    };

public: /* ------------------------------------------ Node's temporary map parameters --------------------------------------------- */

    /// Description of the parameter defining maximal number of markers in the map
    static constexpr node_common::parameters::ParamDescriptor<int> TEMPORARY_MAP_MARKERS_LIMIT_PARAM_DESCRIPTOR {
        .name           = "temporary_map_markers_limit",
        .read_only      = true,
        .dynamic_typing = false,
        .default_value  = 20,
        .description    = "Maximal number of markers in the temporary map"
    };

    /// Description of the parameter defining name of the RVIZ markers namespace associated with last incoming intensity markers
    static constexpr node_common::parameters::ParamDescriptor<std::string> TEMPORARY_MAP_MARKERS_NAMESPACE_PARAM_DESCRIPTOR {
        .name           = "temporary_map_markers_namespace",
        .read_only      = true,
        .dynamic_typing = false,
        .default_value  = "temporary_map_markers",
        .description    = "Name of the RVIZ markers namespace associated with visualized temporary-map markers"
    };

    /// Description of the parameter defining distance threshold for matching markers based on their position
    static constexpr node_common::parameters::ParamDescriptor<std::vector<double>, POSITION_2D_SIZE> TEMPORARY_MAP_DISTANCE_MATCHING_THRESHOLD_PARAM_DESCRIPTOR {
        .name           = "temporary_map_distance_matching_threshold",
        .read_only      = true,
        .dynamic_typing = false,
        .default_value  = std::array{ 0.1, 0.1 },
        .description    = "Array containing { dx, dy } coordinates defining maximal distance between temporary matching markers"
    };

public: /* --------------------------------------- Nodes's compile-time configuration --------------------------------------------- */

    /// Namespace of markers visualizing map intensity markers in RVIZ
    static constexpr auto RVIZ_MAP_MARKERS_NS = "map-markers";
    /// Namespace of markers visualizing temporary map intensity markers in RVIZ
    static constexpr auto RVIZ_TEMPORARY_MAP_MARKERS_NS = "temporary-map-markers";

    /// ID of the points visualizing intensity markers in the RVIZ
    static constexpr auto RVIZ_MARKERS_ID = 1;
    /// XY scale of the points visualizing intensity markers in the RVIZ
    static constexpr auto RVIZ_MARKERS_SCALE_XY = std::array{ 0.1, 0.1 };
    /// RGBA color of the points visualizing map markers in the RVIZ
    static constexpr auto RVIZ_MAP_MARKERS_COLOR_RGBA = std::array{ 0.0f, 1.0f, 0.0f, 1.0f };
    /// RGBA color of the points visualizing temporary map markers in the RVIZ
    static constexpr auto RVIZ_TEMPORARY_MAP_MARKERS_COLOR_RGBA = std::array{ 1.0f, 0.0f, 0.0f, 1.0f };

public: /* ------------------------------------------------ Topic's parameters ---------------------------------------------------- */
    
    // Size of the topics' queue
    static constexpr std::size_t TOPIC_QUEUE_SIZE = 50;

	/// Name of the subscriber interfaces for incoming point cloud scans
	static constexpr auto CLOUD_SUB_TOPIC_NAME = "cloud";

    /// Name of the output topic used to broadcast of points of interest parsed from the environment
    static constexpr auto MARKERS_PUB_TOPIC_NAME = "points_of_interest";
    /// Name of the output topic used to provide current markes map
    static constexpr auto MARKERS_MAP_PUB_TOPIC_NAME = "map";
    /// Name of the output topic on which the RVIZ markers representing the map and the incoming markers are broadcasted
    static constexpr auto VISUALIZATION_PUB_TOPIC_NAME = "visualization";

	/// Name of the service interface used to change node's mode
	static constexpr auto CHANGE_MODE_SRV_TOPIC_NAME = "change_mode";
	/// Name of the service interface used to save current intensity map to the file
	static constexpr auto SAVE_MAP_SRV_TOPIC_NAME = "save_map";
	/// Name of the service interface used to load the intensity map from the file
	static constexpr auto LOAD_MAP_SRV_TOPIC_NAME = "load_map";

public: /* ----------------------------------------------- Public ctors & dtors --------------------------------------------------- */

    /**
     * @brief Construct a new MapBuilder node
     * 
     * @param options 
     *    configuration of the node
     */
    MapBuilder(const rclcpp::NodeOptions & options);

    /**
     * @brief Destroy the MapBuilder node logging goodbye message to the rosout
     */
    ~MapBuilder();

private: /* ------------------------------------------------ Callback methods ----------------------------------------------------- */

    /**
     * @brief Callback for incoming point cloud scans. Depending on the current mode:
     * 
     *      Mapping ) the incoming cloud is used to update both global and temporary map;
     *                also both, the new version of the global and temporary map are published
     *                to the output topics; temporary map is given in the <b>ROBOT</b> frame
     *                of reference 
     * 
     *      Watching ) the incoming cloud is used to update the temporary map; the new version
     *                 of the temporary map is published to the output topics; temporary map 
     *                 is given in the <b>FIXED</b> frame of reference 
     * 
     *       Wapping ) the incoming cloud is used to update both global and temporary map;
     *                also both, the new version of the global and temporary map are published
     *                to the output topics; temporary map is given in the <b>FIXED</b> frame
     *                of reference 
     * 
     */
    void cloud_callback(const sensor_msgs::msg::PointCloud2 &msg);
    
    /**
     * Callback for the change mode service request
     */
    void change_mode_callback(
        const velmwheel_poi_map_builder_msgs::srv::ChangeMode::Request::SharedPtr req,
        velmwheel_poi_map_builder_msgs::srv::ChangeMode::Response::SharedPtr res
    );
    
    /**
     * @brief Callback for the incoming 'save map' service
     * 
     * @param req 
     *    request structure containing name of the output file
     * @param res 
     *    reference to the response structure
     */
    void save_map_callback(
        const poi_map_msgs::srv::SaveMap::Request::SharedPtr req,
        poi_map_msgs::srv::SaveMap::Response::SharedPtr res
    );

    /**
     * @brief Callback for the incoming 'load map' service
     * 
     * @param req 
     *    request structure containing name of the output file
     * @param res 
     *    reference to the response structure
     */
    void load_map_callback(
        const poi_map_msgs::srv::LoadMap::Request::SharedPtr req,
        poi_map_msgs::srv::LoadMap::Response::SharedPtr res
    );

private: /* ----------------------------------------------- Auxiliary methods ----------------------------------------------------- */

    /**
     * @brief Publishes current map with the given timestamp
     * 
     * @param time 
     *    timestamp of the message to be published
     * @param map_reloaded 
     *    value of the @a reloaded attribute in the message to be set
     */
    void publish_map(const rclcpp::Time &time, bool map_reloaded = false);

    /**
     * @brief Callback method called periodically to publish visualization of the current intensity map
     *    along with the visualization of the last received set of intensity markers
     */
    void publish_visualization();

private: /* ------------------------------------------------- ROS interfaces ------------------------------------------------------ */

    /// Cyclical timer used to trigger visualizations' publication
    rclcpp::TimerBase::SharedPtr timer;

	/// Subscriber interfaces for incoming laser scans
	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub;

	/// Publisher interfaces used to provide temporary markers map
	rclcpp::Publisher<poi_map_msgs::msg::MarkersStamped>::SharedPtr markers_pub;
	/// Publisher interface used to provide current markes map
	rclcpp::Publisher<poi_map_msgs::msg::MarkersMap>::SharedPtr markers_map_pub;
	/// Publisher interface used to provide RVIZ vizualization of the output markers
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualization_pub;

	/// Service interface used to change node's mode
	rclcpp::Service<velmwheel_poi_map_builder_msgs::srv::ChangeMode>::SharedPtr change_mode_srv;
	/// Service interface used to request the node to save the current map to the file
	rclcpp::Service<poi_map_msgs::srv::SaveMap>::SharedPtr save_map_srv;
	/// Service interface used to request the node to load the current map from the file
	rclcpp::Service<poi_map_msgs::srv::LoadMap>::SharedPtr load_map_srv;

    /// TF2 buffer utilitized by the @a transform_listener to buffer incoming frames
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    /// TF2 interface listening for current position frames of the mimiced turtle
    std::shared_ptr<tf2_ros::TransformListener> transform_listener{ nullptr };
    
private: /* -------------------------------------------------- Node's state ------------------------------------------------------- */

    /**
     * @enum Mode
     * @brief Enumeration describing node's mode of opration
     */
    enum class Mode : unsigned {
        Idle     = velmwheel_poi_map_builder_msgs::srv::ChangeMode::Request::IDLE,
        Watching = velmwheel_poi_map_builder_msgs::srv::ChangeMode::Request::WATCHING,
        Mapping  = velmwheel_poi_map_builder_msgs::srv::ChangeMode::Request::MAPPING
    };

    // List of valid modes in a numerical form
    static constexpr auto VALID_MODES = std::array { 
        static_cast<unsigned>( Mode::Idle     ),
        static_cast<unsigned>( Mode::Watching ),
        static_cast<unsigned>( Mode::Mapping  )
    };

    /// Current mode of operation
    Mode mode { Mode::Idle };

    /**
     * @note Underlying members are stored inside the @ref std::optional object to postpone
     *   it's construction until node's parameters are acquired from the parameters server
     */

    /// Underlying object managing actual building of the map
    std::optional<MapBuilderImpl> map_builder;

    /// Object used to visualize map-markers in the RVIZ-acceptable form
    std::optional<poi_map::MapVisualizer> map_markers_visualizer;
    /// Object used to visualize temporary-map-markers in the RVIZ-acceptable form
    std::optional<poi_map::MapVisualizer> temporary_map_markers_visualizer;

    /**
     * @see MapBuilderImpl's description to get known what's the difference between
     *    map and temporary-map
     */
    
};

/* ================================================================================================================================ */

} // End namespace velmwheel::points_of_interest

#endif
