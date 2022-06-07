/* ============================================================================================================================ *//**
 * @file       markers_visualizer.cpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Saturday, 2nd April 2022 5:39:15 pm
 * @modified   Wednesday, 25th May 2022 5:44:37 pm
 * @project    engineering-thesis
 * @brief      Definition of the poi_map::MapVisualizer class providing conversion between vectors of poi_map_msgs::Marker
 *             and RVIZ-specific visualization_msgs::msg::Marker structure
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

/* =========================================================== Includes =========================================================== */

// System includes
#include <exception>
// Private includes
#include "poi_map/map_visualizer.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace poi_map {

/* ======================================================== Static methods ======================================================== */

MapVisualizerConstructor MapVisualizer::make_constructor() {
    return MapVisualizerConstructor{};
}

/* ========================================================= Public ctors ========================================================= */

MapVisualizer::MapVisualizer(
    double score_threshold,
    const std::string &ns,
    int32_t id,
    const rclcpp::Duration &duration,
    const std::array<double, 2> &scale_xy,
    const std::array<float, 4> &color_rgba
) :
    score_threshold { score_threshold }
{

    // Fill body of the Marke message
    msg.ns           = ns;
    msg.id           = id;
    msg.type         = visualization_msgs::msg::Marker::POINTS;
    msg.action       = visualization_msgs::msg::Marker::MODIFY;
    msg.lifetime     = duration;
    msg.scale.x      = scale_xy[0];
    msg.scale.y      = scale_xy[1];
    msg.frame_locked = true;

    // Cache target color
    color.r = color_rgba[0];
    color.g = color_rgba[1];
    color.b = color_rgba[2];
    color.a = color_rgba[3];

}

/* ======================================================== Public methods ======================================================== */

visualization_msgs::msg::Marker MapVisualizer::visualize(
    const rclcpp::Time &stamp,
    const std::string &frame_id,
    const std::vector<poi_map_msgs::msg::Marker>& markers
) {
    
    // CLeanup the sceleton message
    msg.points.clear();

    // Fill message's header
    msg.header.stamp    = stamp;
    msg.header.frame_id = frame_id;

    // Add points to the visualization
    for(auto &marker : markers) {

        // Skipp points with too small score
        if(marker.score > score_threshold) {

            // Add point to the visualization
            msg.points.emplace_back();
            // Set coordinates of the added point
            msg.points.back().x = marker.position.x;
            msg.points.back().y = marker.position.y;
            msg.points.back().z = 0.0;

            // Add colour of the added point to the visualization
            msg.colors.emplace_back(color);
            
        }
    }

    // Return the message (move it to avoid copying vectors owned by the message)
    return std::move(msg);
}

/* ======================================================= Constructor class ====================================================== */

MapVisualizerConstructor &MapVisualizerConstructor::score_threshold(double threshold) {
    this->score_threshold_ = threshold;
    return *this;
}


MapVisualizerConstructor &MapVisualizerConstructor::ns(const std::string &ns) {
    this->ns_ = ns;
    return *this;
}


MapVisualizerConstructor &MapVisualizerConstructor::id(int32_t id) {
    this->id_ = id;
    return *this;
}


MapVisualizerConstructor &MapVisualizerConstructor::duration(const rclcpp::Duration &duration) {
    this->duration_ = duration;
    return *this;
}


MapVisualizerConstructor &MapVisualizerConstructor::scale_xy(const std::array<double, 2> &scale) {
    this->scale_xy_ = scale;
    return *this;
}


MapVisualizerConstructor &MapVisualizerConstructor::color_rgba(const std::array<float, 4> &color) {
    this->color_rgba_ = color;
    return *this;
}


MapVisualizer MapVisualizerConstructor::operator*() {
    
    // Check if all parameters has been set
    bool properly_initialized =
        score_threshold_.has_value() and
        ns_.has_value()              and
        id_.has_value()              and
        duration_.has_value()        and
        scale_xy_.has_value()        and
        color_rgba_.has_value();

    // Throw error if not
    if(not properly_initialized)
        throw std::runtime_error{ "Cannot generate poi_map::MapVisualizer with lacking parameters" };

    // Create the builder object
    return MapVisualizer(
        *score_threshold_,
        *ns_,
        *id_,
        *duration_,
        *scale_xy_,
        *color_rgba_
    );

}

/* ================================================================================================================================ */

} // End namespace poi_map

/* ================================================================================================================================ */
