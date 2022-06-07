/* ============================================================================================================================ *//**
 * @file       poi_map_builder_impl.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 31st March 2022 9:16:44 pm
 * @modified   Thursday, 26th May 2022 12:03:40 am
 * @project    engineering-thesis
 * @brief      Declaration of the ROS2 component node class building an intensity map of points of interest (POI)
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __VELMWHEEL_POI_MAP_BUILDER_IMPL_H__
#define __VELMWHEEL_POI_MAP_BUILDER_IMPL_H__

/* =========================================================== Includes =========================================================== */

// System includes
#include <array>
#include <string>
#include <memory>
// ROS includes
#include "rclcpp/rclcpp.hpp"
// Interfaces includes
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "poi_map_msgs/msg/marker.hpp"
// TF includes
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
// Private includes
#include "poi_map/map_loader.hpp"
#include "poi_map/map_builder.hpp"
#include "velmwheel/poi_map_strategies.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace velmwheel::points_of_interest {

/* ======================================================== Auxiliary class ======================================================= */

class MapBuilderImpl;

/**
 * @brief Auxiliary view class constituting non-owning view over the intensity map
 *    enabling transforming it into an arbitrary reference frame
 */
class MapView {

public: /* -------------------------------------------------- Public methods ------------------------------------------------------ */

    /**
     * @param target_frame 
     *    target reference frame
     * @returns 
     *    copy of the viewed intensity map transformed into the @p target_frame
     */
    std::vector<poi_map_msgs::msg::Marker> in_frame(const std::string &target_frame);

    /**
     * @returns 
     *    reference to the viewed map in the original frame of reference
     */
    operator std::vector<poi_map_msgs::msg::Marker>();

private: /* --------------------------------------------- Private ctors & dtors --------------------------------------------------- */

    /**
     * @brief Construct a new POIMapView object
     * 
     * @param tf_buffer 
     *    reference to the instance of the TF buffer that can be used to obtain 
     *    transformation between two frames
     * @param map 
     *    reference to the viewed map
     * @param reference_frame 
     *    reference frame of the viewed map
     * @param stamp 
     *    timestmap of the viewed map
     */
    MapView(
        tf2_ros::Buffer &tf_buffer,
        const std::vector<poi_map_msgs::msg::Marker> &map,
        const std::string &reference_frame,
        const rclcpp::Time &stamp
    );

    // Let MapBuilderImpl build the MapView
    friend class MapBuilderImpl;

private: /* ------------------------------------------------- Object's state ------------------------------------------------------ */

    /// Reference to the instance of the TF buffer that can be used to obtain transformation between two frames
    tf2_ros::Buffer &tf_buffer;

    /// Reference to the viewed map
    const std::vector<poi_map_msgs::msg::Marker> &map;
    /// Frame of reference of the map
    const std::string reference_frame;
    /// Timestamp of the map
    const rclcpp::Time stamp;

};

/* ============================================================= Class ============================================================ */

class MapBuilderConstructor;

/**
 * @brief Class managing building of the map of points-of-interest (POI) present in the environment
 *    of the robot based on incoming scans of this environment
 * @details MapBuilderImpl class provides a complete subsystem for building a map of characteristic points
 *    (points-of-interest, POI) present in the environment of the robot basing on it's 2D sensorical
 *    representation [1]. Constructed object holds two types of maps named 'global map' ('map' for short)
 *    and 'temporary' map. Both object store set of markers representing points in 3D space described
 *    by a single floating-point value named 'intensity' These represent points in the real environment
 *    that have been observed by scanning sensors and has been classified as POI. Set of such markers
 *    can be utilized by high-level nodes to provide global localization of the robot in the environment
 *    [2].
 * 
 *    Incoming data is optionally transformed to the robot's frame of reference as neeeded and transformed
 *    into the markers. Next it is initially filtered [3]. Preprocessed data serves as a direct input to
 *    the builder block of the first map.
 * 
 *    In general sense, the building algorithm is identical for boths maps and it is based on two-step 
 *    workflow. Firstly, it browses incoming markers and selects those that meet criterions [4] to be
 *    classified as POI. Next, the builder compares tipped markers with those currently saved in the
 *    map. If an aspirating point meets 'matching' condition [5], the 'intensity' of the corresponding 
 *    point in the map is increased [6]. Also the position of the saved marker is corrected to better
 *    match observed position [7]. Otherwise, the point is added to the map (assuming that limit of markers
 *    has not been reached) with 'intensity' equal to @c 0 . Finally, the map is subjected to the 
 *    'discarding' process that aims to remove markers considered out-of-date from the map [7].
 * 
 *    Difference between both maps lies in:
 * 
 *       1) details of the algorithm's steps (descibed shortly in [x] annotations)
 *       2) interpretation of stored markers
 * 
 *    The 'temporary map' is a set of POI observed by sensors in the near past. Elements of this map 
 *    are expected to be often removed from the set what corresponds to the POI disappearing from the
 *    field of view of the moving robot [8]. This map is builded based on the markers obtained directly
 *    from incoming data.
 *    
 *    The 'global map' ('map') stores long-term POI observed in the whole environment observed by the 
 *    robot. This map can be saved and loaded to/from the disk storage between activity cycles of the 
 *    robot. Markers in map are rarely discarded. Input data for the bulding subsystem responsible for
 *    this map is set of markers present in the 'temporary' map.
 * 
 * @note [1] Currently only point-cloud input data representation is supported, to match usecase
 *    of the WUT Velmwheel ROS system, but the implementation can be easily extended to support other
 *    representations, e.g. raw laser scans.
 * @note [2] The current implementation assummes that the environment of the robot is aproximately 
 *    non-evolutioning. However, thanks to a <i>small-corrections mechanism</i> utilized by the mapping
 *    algorithm, the subsystem can prove working in the slow- or rarely-evolving setting. 
 * @note [3] Current implementation of the filtering aims only to dense scan data by removing
 *    points too loosely positioned to each other.
 * @note [4] For the 'temporary map' the criterion is based on the concept of so called regions-of-interest
 *    (ROI). These are segments of the scanned space whose in which 'intensity' of all points included
 *    is higher than the predefined threshold. POI points are choosen as centres of those segments.
 *    For the 'global map' the selection criterion is currently unused. 
 * @note [5] Current implementation assumes slow-moving model of the robot. For that reason points are matched
 *    based on their estimated distance. Points are considered corresponding if distance between them does not
 *    cross the predefined threshold. At the moment, the rectilinear metric is used to measure the distance.
 * @note [6] In case of both maps, intensity of the 'matched' is incremented by @c 1 for each match.
 * @note [7] In case of both maps, the <i>small-corrections mechanism</i> moves the saved marker to the midpoint
 *    between the moved marker and the observed marker
 * @note [8] At the moment this step is implemented in neither of maps ( @todo implement the 'discarding' mechanism)
 * 
 * @note All elements of the subsystem described by [x] annotations has been implemented as 
 *    'injectedly-dependent' (wing-wing) elements of the wider abstract algorithm. They are intended
 *    to be easily interchangable depending on the requirements of the target system.
 * 
 * @see <i>/doc</i> directory to obtain block diagram of the subsystem
 */
class MapBuilderImpl {

public: /* -------------------------------------------------- Static methods ------------------------------------------------------ */

    /**
     * @brief Creates a 'builder' class for the MapBuilderImpl
     */
    static MapBuilderConstructor make_constructor();

public: /* ----------------------------------------------- Public ctors & dtors --------------------------------------------------- */

    /**
     * @brief Construct a new MapBuilderImpl object
     * 
     * @param tf_buffer 
     *    TF2 buffer utilitized to obtain transformations between frames
     * @param distance_limit
     *    maximal distance between subsequent points to be processed in [m] 
     * @param intensity_threshold 
     *    threshold intensity of the robot-observed point to be considered as a boundary of the ROI (region of intereset)
     * @param fixed_frame 
     *    name of the fixed frame of reference for markers in the resulting map
     * @param robot_frame 
     *    name of the robot's local TF frame
     * @param intensity_limit 
     *    intensity limit of the map's markers
     * @param save_intensity_threshold 
     *    threshold value determining minimal intensity of the marker to not being discarded when storing map into XML file
     * @param map_markers_limit 
     *    maximal number of markers in the map
     * @param map_distance_matching_threshold_xy 
     *    array containing { dx, dy } coordinates defining maximal distance between temporary-map markers and map markes 
     *    to be considered matching
     * @param temporary_map_markers_limit 
     *    maximal number of markers in the temporary map
     * @param temporary_map_distance_matching_threshold_xy 
     *    array containing { dx, dy } coordinates defining maximal distance between temporary matching markers
     */
    MapBuilderImpl(
        tf2_ros::Buffer &tf_buffer,
        double distance_limit,
        double intensity_threshold,
        const std::string &fixed_frame,
        const std::string &robot_frame,
        double intensity_limit,
        double save_intensity_threshold,
        std::size_t map_markers_limit,
        const std::array<double, 2> &map_distance_matching_threshold_xy,
        std::size_t temporary_map_markers_limit,
        const std::array<double, 2> &temporary_map_distance_matching_threshold_xy
    );

    /// Default copy-constructor
    MapBuilderImpl(const MapBuilderImpl &other) = default;
    /// Default move-constructor
    MapBuilderImpl(MapBuilderImpl &&other) = default;

public: /* ------------------------------------------- Public methods (maps update) ----------------------------------------------- */

    /**
     * @brief Updates both temporary and global map based on the given cloud of points
     * 
     * @param msg 
     *    point cloud of the scanned environment containing informations about point's
     *    position and intensity
     */
    void update_map(const sensor_msgs::msg::PointCloud2 &msg);

    /**
     * @brief Updates temporary map based on the given cloud of points
     * 
     * @param msg 
     *    point cloud of the scanned environment containing informations about point's
     *    position and intensity
     */
    void update_temporary_map(const sensor_msgs::msg::PointCloud2 &msg);

public: /* ------------------------------------------ Public methods (maps viewing) ----------------------------------------------- */

    /**
     * @returns 
     *    name of the preconfigured fixed reference frame
     */
    const std::string &get_fixed_frame() const;

    /**
     * @returns 
     *    name of the preconfigured robot reference frame
     */
    const std::string &get_robot_frame() const;

    /**
     * @param stamp 
     *    timestamp of the requested map 
     * @returns 
     *    reference to the current temporary map represented in the fixed
     *    frame of reference
     */
    MapView get_temporary_map(const rclcpp::Time &stamp) const;

    /**
     * @param stamp 
     *    timestamp of the requested map 
     * @returns 
     *    reference to the current global map represented in the fixed
     *    frame of reference
     */
    MapView get_map(const rclcpp::Time &stamp) const;

public: /* --------------------------------------- Public methods (maps manipulation) --------------------------------------------- */

    /**
     * @brief Saves global map to the XML file
     * 
     * @param filename 
     *    name of the target file
     */
    void save_map(const std::string &filename) const;

    /**
     * @brief Loads global map from the file
     * 
     * @param filename 
     *    name of the source file
     */
    void load_map(const std::string &filename);
    
private: /* --------------------------------------------- Object's configuration -------------------------------------------------- */

    /// TF2 buffer utilitized to obtain transformations between frames
    tf2_ros::Buffer &tf_buffer;
    /// Name of the TF frame being a reference point of the global map
    const std::string fixed_frame;
    /// Name of the TF frame related to the local coordinate system of the robot being a reference point of the temporary map
    const std::string robot_frame;

private: /* ------------------------------------------------- Object's state ------------------------------------------------------ */

    /// Input data filer
    std::unique_ptr<DataFilteringStrategy> data_filter;

    /// Object managing building the temporary intensity map
    poi_map::MapBuilder temporary_map_builder;
    /// Object managing building the intensity map
    poi_map::MapBuilder map_builder;
    /// Auxiliary object managing storing intensity map to the file
    poi_map::MapLoader map_loader;

};

/* ==================================================== Auxiliary builder class =================================================== */

/**
 * @brief Builder class for the MapBuilderImpl
 */
class MapBuilderConstructor {

public: /* ----------------------------------------------- Public ctors & dtors --------------------------------------------------- */

    /**
     * @brief Construct a new MapBuilderConstructor object
     */
    MapBuilderConstructor() = default;

public: /* -------------------------------------------------- Public setters ------------------------------------------------------ */

    /// @brief Sets target intensity threshold 
    MapBuilderConstructor &tf_buffer(tf2_ros::Buffer &buffer);

    /// @brief Sets target distance limit 
    MapBuilderConstructor &distance_limit (double limit);
    /// @brief Sets target intensity threshold 
    MapBuilderConstructor &intensity_threshold (double threshold);
    /// @brief Sets target fixed frame
    MapBuilderConstructor &fixed_frame (const std::string &frame);
    /// @brief Sets target robot frame
    MapBuilderConstructor &robot_frame (const std::string &frame);
    /// @brief Sets target intensity limit
    MapBuilderConstructor &intensity_limit (double limit);
    /// @brief Sets target save intensity threshold
    MapBuilderConstructor &save_intensity_threshold (double threshold);

    /// @brief Sets target map markers limit
    MapBuilderConstructor &map_markers_limit (std::size_t limit);
    /// @brief Sets target map distance matching threshold
    MapBuilderConstructor &map_distance_matching_threshold_xy (const std::array<double, 2> &threshold);
    
    /// @brief Sets target temporary map markers limit
    MapBuilderConstructor &temporary_map_markers_limit (std::size_t limit);
    /// @brief Sets target temporary map distance matching threshold
    MapBuilderConstructor &temporary_map_distance_matching_threshold_xy (const std::array<double, 2> &temporary_threshold);

public: /* -------------------------------------------------- Public methods ------------------------------------------------------ */

    /**
     * @brief Constructs the MapBuilderImpl object
     * 
     * @throw std::runtime_error
     *    if any of non-default parameters has not been configured
     */
    MapBuilderImpl operator*() const;

private: /* ------------------------------------------------- Object's state ------------------------------------------------------ */

    /// Target TF buffer
    tf2_ros::Buffer *tf_buffer_ { nullptr };

    /// Target distancelimit 
    std::optional<double> distance_limit_;
    /// Target intensity threshold 
    std::optional<double> intensity_threshold_;
    /// Target fixed frame
    std::optional<std::string> fixed_frame_;
    /// Target robot frame
    std::optional<std::string> robot_frame_;
    /// Target intensity limit
    std::optional<double> intensity_limit_;
    /// Target save intensity threshold
    std::optional<double> save_intensity_threshold_;

    /// Target map markers limit
    std::optional<std::size_t> map_markers_limit_;
    /// Target map distance matching threshold
    std::optional<std::array<double, 2>> map_distance_matching_threshold_xy_;
    
    /// Target temporary map markers limit
    std::optional<std::size_t> temporary_map_markers_limit_;
    /// Target temporary map distance matching threshold
    std::optional<std::array<double, 2>> temporary_map_distance_matching_threshold_xy_;
};

/* ================================================================================================================================ */

} // End namespace velmwheel::points_of_interest

#endif
