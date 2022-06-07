/* ============================================================================================================================ *//**
 * @file       map_visualizer.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 31st March 2022 3:27:14 pm
 * @modified   Wednesday, 25th May 2022 5:44:06 pm
 * @project    engineering-thesis
 * @brief      Declaration of the poi_map::MapVisualizer class providing conversion between vectors of poi_map_msgs::Marker
 *             and RVIZ-specific visualization_msgs::msg::Marker structure
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __POI_MAP_MAP_VISUALIZER_H__
#define __POI_MAP_MAP_VISUALIZER_H__

/* =========================================================== Includes =========================================================== */

// System includes
#include <string>
#include <optional>
#include <memory>
// ROS includes
#include "rclcpp/time.hpp"
// Interfaces includes
#include "std_msgs/msg/color_rgba.hpp"
#include "visualization_msgs/msg/marker.hpp"
// Private includes
#include "poi_map_msgs/msg/marker.hpp"

/* ================================================================================================================================ */

namespace poi_map {

/* ============================================================= Class ============================================================ */

class MapVisualizerConstructor;

/**
 * @brief Class providing conversion between vectors of poi_map_msgs::Marker
 *    and RVIZ-specific visualization_msgs::msg::Marker structure
 * 
 * @todo At the moment class visualizes list of markers as a single 
 *    visualization_msgs::msg::Marker object of type POINTS. Also all of points
 *    represented by such a marker are monochromatic and placed on the Z=0 plane.
 *    In the future it would be helpfull to provide strategy-like dependecy injection
 *    for customizing color and/or height of RVIZ-markers depending on the score
 *    of transformed markers 
 */
class MapVisualizer {

public: /* ------------------------------------------------- Static methods ------------------------------------------------------- */

    /**
     * @brief Creates the 'builder-like' object for constructing the MapVisualizer object
     */
    static MapVisualizerConstructor make_constructor();

public: /* ----------------------------------------------- Public ctors & dtors --------------------------------------------------- */

    /**
     * @brief Construct a new Markers Visualizer object
     * 
     * @param score_threshold 
     *    threshold score under which markers are not visualized
     * @param ns 
     *    reference namespace for the target RVIZ marker
     * @param id 
     *    ID of the target RVIZ marker
     * @param duration 
     *    duration of the target RVIZ marker
     * @param scale_xy 
     *    scale of points in the target RVIZ marker
     * @param color_rgba 
     *    color of points in the target RVIZ marker
     */
    MapVisualizer(
        double score_threshold,
        const std::string &ns,
        int32_t id,
        const rclcpp::Duration &duration,
        const std::array<double, 2> &scale_xy,
        const std::array<float, 4> &color_rgba
    );

    /**
     * @brief Default copy constructor
     */
    MapVisualizer(const MapVisualizer &other) = default;

    /**
     * @brief Default move constructor
     */
    MapVisualizer(MapVisualizer &&other) = default;

public: /* -------------------------------------------------- Public methods ------------------------------------------------------ */

    /**
     * @brief Visualzies given vector of PoI @p markers
     * 
     * @param stamp 
     *    timestamp of the resulting message
     * @param frame_id 
     *    frame_id of the resulting message
     * @param markers 
     *    set of markers to be visualized
     */
    visualization_msgs::msg::Marker visualize(
        const rclcpp::Time &stamp,
        const std::string &frame_id,
        const std::vector<poi_map_msgs::msg::Marker>& markers
    );


public: /* -------------------------------------------------- Object's state ------------------------------------------------------ */

    /// Threshold score under which markers are not visualized
    double score_threshold;

    /// Skeleton of the target visualization
    visualization_msgs::msg::Marker msg;

    /// Target color for visualized markers
    std_msgs::msg::ColorRGBA color;

};

/* ======================================================= Auxiliary classes ====================================================== */

/**
 * @brief Auxiliary 'builder' class for the MapVisualizer
 */
class MapVisualizerConstructor {
    
public:

    /**
     * @brief Constructs a new MapVisualizerConstructor object
     */
    MapVisualizerConstructor() = default;

public:
    
    /// @brief Sets threshold score under which markers are not visualized
    MapVisualizerConstructor & score_threshold(double threshold);
    /// @brief Sets reference namespace for the target RVIZ marker
    MapVisualizerConstructor &ns(const std::string &ns);
    /// @brief Sets ID of the target RVIZ marker
    MapVisualizerConstructor &id(int32_t id);
    /// @brief Sets duration of the target RVIZ marker
    MapVisualizerConstructor &duration(const rclcpp::Duration &duration);
    /// @brief Sets scale of points in the target RVIZ marker
    MapVisualizerConstructor &scale_xy(const std::array<double, 2> &scale);
    /// @brief Sets color of points in the target RVIZ marker
    MapVisualizerConstructor &color_rgba(const std::array<float, 4> &color);
    
public:

    /**
     * @brief Builds the MapVisualizer class
     * 
     * @throws std::runtime_error
     *    if any of MapVisualizer's parameters has not been initialized
     */
    MapVisualizer operator*();

private:

    /// Target threshold score under which markers are not visualized
    std::optional<double> score_threshold_;
    /// Target reference namespace for the target RVIZ marker
    std::optional<std::string> ns_;
    /// Target ID of the target RVIZ marker
    std::optional<int32_t> id_;
    /// Target duration of the target RVIZ marker
    std::optional<rclcpp::Duration> duration_;
    /// Target scale of points in the target RVIZ marker
    std::optional<std::array<double, 2>> scale_xy_;
    /// Target color of points in the target RVIZ marker
    std::optional<std::array<float, 4>> color_rgba_;

};

/* ================================================================================================================================ */

} // End namespace poi_map

/* ================================================================================================================================ */

#endif
