/* ============================================================================================================================ *//**
 * @file       builder.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 31st March 2022 7:41:36 pm
 * @modified   Wednesday, 25th May 2022 4:50:30 pm
 * @project    engineering-thesis
 * @brief      Definitions of the poi_map::Builder class implementing routines focusing on building an PoI map
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __POI_MAP_BUILDER_BUILDER_H__
#define __POI_MAP_BUILDER_BUILDER_H__

/* =========================================================== Includes =========================================================== */

// System includes
#include <algorithm>
#include <exception>
// TF includes
#include "tf2/LinearMath/Vector3.h"
// Private includes
#include "poi_map/builder.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace poi_map {
    
/* ========================================================= Helper macros ======================================================== */

/**
 * @brief Helper macro defining the (too long, too ugly) template list
 *    of the Builder class
 */
#define BuilderTemplateList                   \
        typename PointsMatchingStrategy,      \
        typename ScoreUpdateStrategy,         \
        typename PositionUpdateStrategy,      \
        typename RepresentationHeightStrategy

/**
 * @brief Helper macro defining the (too long, too ugly) template arguments list
 *    of the Builder class
 */
#define BuilderTemplateArgList       \
        PointsMatchingStrategy,      \
        ScoreUpdateStrategy,         \
        PositionUpdateStrategy,      \
        RepresentationHeightStrategy

/**
 * @brief Helper macro defining the (too long, too ugly) template header
 *    of methods of the Builder class
 * @param ...
 *    return type
 */
#define BuilderSignature(...)       \
    template<BuilderTemplateList>   \
    __VA_ARGS__                     \
    Builder<BuilderTemplateArgList>

/**
 * @brief Helper macro defining the (too long, too ugly) template header
 *    of methods of the BuilderConstructor class
 * @param ...
 *    return type
 */
#define BuilderConstructorMethodSignature(...) \
    template<BuilderTemplateList>              \
    __VA_ARGS__                                \
    BuilderConstructor<BuilderTemplateArgList>

/**
 * @brief Helper macro defining a signature for the BuilderConstructor
 *    class' method
 */
#define BuilderConstructorSetterSignature            \
    BuilderConstructorMethodSignature(               \
        BuilderConstructor<BuilderTemplateArgList> & \
    )

/* ======================================================== Static methods ======================================================== */

BuilderSignature(BuilderConstructor<BuilderTemplateArgList>)::make_constructor() {
    return BuilderConstructor<BuilderTemplateArgList>{ };
}
    
/* ========================================================= Public ctors ========================================================= */

BuilderSignature()::Builder(
    std::size_t markers_limit,
    const std::string &fixed_frame,
    const std::string &robot_frame,
    const std::string &map_markers_namespace,
    const std::string &incoming_markers_namespace,
    const std::array<float, 4> &map_markers_color,
    const std::array<float, 4> &incoming_markers_color,
    const PointsMatchingStrategy &points_matching_strategy,
    const ScoreUpdateStrategy &score_update_strategy,
    const PositionUpdateStrategy &position_update_strategy,
    const RepresentationHeightStrategy &representation_height_strategy
) :
    fixed_frame { fixed_frame },
    robot_frame { robot_frame },
    map_markers_namespace { map_markers_namespace },
    incoming_markers_namespace { incoming_markers_namespace },
    map_markers_color { map_markers_color },
    incoming_markers_color { incoming_markers_color },
    points_matching_strategy { points_matching_strategy },
    score_update_strategy { score_update_strategy },
    position_update_strategy { position_update_strategy },
    representation_height_strategy { representation_height_strategy }
{
    // Reserve storage for the local PoI map
    map.reserve( markers_limit );
}

/* ======================================================== Public methods ======================================================== */

BuilderSignature(void)::update_map(
    const std::vector<poi_map_msgs::msg::Marker>& markers,
    const tf2::Transform &fixed_to_robot_transform
) {

    // Copy incoming markers into the local buffer
    std::vector<poi_map_msgs::msg::Marker> markers_in_fixed_frame { markers };

    // Transform markers' positions into the fixed frame of reference
    for(auto &marker : markers_in_fixed_frame) {

        // Parse marker's position into the TF vector
        tf2::Vector3 position {
            marker.position.x,
            marker.position.y,
            0.0
        };

        // Transform position into the fixed frame of reference
        position = fixed_to_robot_transform(position);
        // Set marker's position in the new frame of reference
        marker.position.x = position.x();
        marker.position.y = position.y();
        
    }

    // Iterate over incoming markers
    for(auto &marker : markers_in_fixed_frame) {

        // Try to find the corresponding marker in the current PoI map
        auto corresponding_marker_in_map = std::find_if( map.begin(), map.end(),
            [this, &marker](const auto &m) { return points_matching_strategy.match(m, marker); }
        );

        // If the corresponding marker has been found
        if(corresponding_marker_in_map != map.end()) {

            // Update PoI fo the marker
            corresponding_marker_in_map->score = score_update_strategy.update_score(*corresponding_marker_in_map, marker);
            // Update position fo the marker
            corresponding_marker_in_map->position = position_update_strategy.update_position(*corresponding_marker_in_map, marker);

        // If the corresponding marker has not been found
        } else {

            // If a new marker fits into the map
            if(map.size() < map.capacity()) {

                // Add a new marker in the map
                map.push_back(marker);
                // Reset marker's score
                map.back().score = 0;
                
            }

        }

    }

    // Cache the incoming set of markers
    last_markers_received = markers;
}

BuilderSignature(const std::vector<poi_map_msgs::msg::Marker> &)::get_map() const {
    return map;
}


BuilderSignature(void)::set_map(const std::vector<poi_map_msgs::msg::Marker> &map) {
    this->map = map;
}


BuilderSignature(visualization_msgs::msg::Marker)::get_map_visualization(double score_threshold) const {
    return markers_to_visualization(
        map,
        map_markers_color,
        fixed_frame,
        map_markers_namespace,
        score_threshold
    );
}


BuilderSignature(visualization_msgs::msg::Marker)::get_current_markers_visualization(double score_threshold) const {
    return markers_to_visualization(
        last_markers_received,
        incoming_markers_color,
        robot_frame,
        incoming_markers_namespace,
        score_threshold
    );
}

/* ================================================= Public visualization methods ================================================= */

BuilderSignature(visualization_msgs::msg::Marker)::markers_to_visualization(
    const std::vector<poi_map_msgs::msg::Marker>& markers,
    const std::array<float, 4> &rgba_color, 
    const std::string &reference_frame,
    const std::string &ns,
    double score_threshold
) const {

    visualization_msgs::msg::Marker ret;

    // Fill header of the Marke message
    ret.header.frame_id = reference_frame;
    // Fill body of the Marke message
    ret.ns           = ns;
    ret.id           = 1;
    ret.type         = visualization_msgs::msg::Marker::POINTS;
    ret.action       = 0;
    ret.lifetime     = rclcpp::Duration(0);
    ret.scale.x      = 0.1;
    ret.scale.y      = 0.1;
    ret.frame_locked = true;

    // Add points to the visualization
    for(auto &marker : markers) {

        // Skipp points with too small score
        if(marker.score > score_threshold) {

            // Add point to the visualization
            ret.points.emplace_back();
            // Set coordinates of the added point
            ret.points.back().x = marker.position.x;
            ret.points.back().y = marker.position.y;
            ret.points.back().z = representation_height_strategy.height_of(marker);

            // Add colour of the added point to the visualization
            ret.colors.emplace_back();
            // Set coordinates of the added point
            ret.colors.back().r = rgba_color[Color::R];
            ret.colors.back().g = rgba_color[Color::G];
            ret.colors.back().b = rgba_color[Color::B];
            ret.colors.back().a = rgba_color[Color::A];
            
        }
    }

    return ret;
}

/* ======================================================= Constructor class ====================================================== */

BuilderConstructorSetterSignature::markers_limit(std::size_t limit) {
    markers_limit_.emplace(limit);
    return *this; 
}


BuilderConstructorSetterSignature::fixed_frame(const std::string& frame) {
    fixed_frame_.emplace(frame);
    return *this; 
}


BuilderConstructorSetterSignature::robot_frame(const std::string& frame) {
    robot_frame_.emplace(frame);
    return *this; 
}


BuilderConstructorSetterSignature::map_markers_namespace(const std::string& ns) {
    map_markers_namespace_.emplace(ns);
    return *this; 
}


BuilderConstructorSetterSignature::incoming_markers_namespace(const std::string& ns) {
    incoming_markers_namespace_.emplace(ns);
    return *this; 
}


BuilderConstructorSetterSignature::map_markers_color(const std::array<float, 4>& rgba_color) {
    map_markers_color_.emplace(rgba_color);
    return *this; 
}


BuilderConstructorSetterSignature::incoming_markers_color(const std::array<float, 4>& rgba_color) {
    incoming_markers_color_.emplace(rgba_color);
    return *this; 
}


BuilderConstructorSetterSignature::points_matching_strategy(const PointsMatchingStrategy& strategy) {
    points_matching_strategy_.emplace(strategy);
    return *this; 
}


BuilderConstructorSetterSignature::score_update_strategy(const ScoreUpdateStrategy& strategy) {
    score_update_strategy_.emplace(strategy);
    return *this; 
}


BuilderConstructorSetterSignature::position_update_strategy(const PositionUpdateStrategy& strategy) {
    position_update_strategy_.emplace(strategy);
    return *this; 
}


BuilderConstructorSetterSignature::representation_height_strategy(const RepresentationHeightStrategy& strategy) {
    representation_height_strategy_.emplace(strategy);
    return *this; 
}


BuilderConstructorMethodSignature(Builder<BuilderTemplateArgList>)::operator*() const {

    // Check if all parameters has been set
    if(
        not markers_limit_.has_value()                  or
        not fixed_frame_.has_value()                    or
        not robot_frame_.has_value()                    or
        not map_markers_namespace_.has_value()          or
        not incoming_markers_namespace_.has_value()     or
        not map_markers_color_.has_value()              or
        not incoming_markers_color_.has_value()         or
        not points_matching_strategy_.has_value()       or
        not score_update_strategy_.has_value()          or
        not position_update_strategy_.has_value()       or
        not representation_height_strategy_.has_value()
    )
        throw std::runtime_error{ "Cannot generate poi_map::Builder with lacking parameters" };

    // Create the builder object
    return Builder<
        PointsMatchingStrategy,
        ScoreUpdateStrategy,
        PositionUpdateStrategy,
        RepresentationHeightStrategy 
    >(
        *markers_limit_,
        *fixed_frame_,
        *robot_frame_,
        *map_markers_namespace_,
        *incoming_markers_namespace_,
        *map_markers_color_,
        *incoming_markers_color_,
        *points_matching_strategy_,
        *score_update_strategy_,
        *position_update_strategy_,
        *representation_height_strategy_
    );
}

/* ================================================================================================================================ */

#undef BuilderTemplateList
#undef BuilderTemplateArgList
#undef BuilderSignature
#undef BuilderConstructorMethodSignature
#undef BuilderConstructorSetterSignature

/* ================================================================================================================================ */

} // End namespace poi_map

/* ================================================================================================================================ */

#endif
