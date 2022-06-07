/* ============================================================================================================================ *//**
 * @file       poi_map_strategies.cpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 31st March 2022 7:41:36 pm
 * @modified   Thursday, 26th May 2022 12:14:04 am
 * @project    engineering-thesis
 * @brief      Definitions of strategies associated with the points of interest (POI) map builder for the WUT Velmwheel robot
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

/* =========================================================== Includes =========================================================== */

// System includes
#include <cmath>
// Private includes
#include "velmwheel/poi_map_strategies.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace velmwheel::points_of_interest {

/* ======================================================== DenseDataFilter ======================================================= */

DenseDataFilter::DenseDataFilter(double max_distance) :
    max_distance { max_distance }
{ }

std::vector<poi_map_msgs::msg::Marker> DenseDataFilter::filter(const std::vector<poi_map_msgs::msg::Marker> &data) {

    std::vector<poi_map_msgs::msg::Marker> ret;

    // Auxiliary functor calculating distance between two points
    auto classified = [this](const poi_map_msgs::msg::Marker &p, const poi_map_msgs::msg::Marker &q) {
        return (std::sqrt( std::pow(p.position.x - q.position.x, 2) + std::pow(p.position.y - q.position.y, 2) ) <= max_distance);
    };

    // If map contains one element, return it
    if(data.size() == 1)
        return data;
    // Else, if data contain two elements, return both conditionally
    else if(data.size() == 2) {
        if(classified(data[0], data[1]))
            return data;
        else
            return ret;
    }

    // Iterate over subsequent points
    for(std::size_t i = 0; i < data.size() - 1; ++i) {

        // If two subsequent points are close enough to each other, add the first point to the return set
        if(classified(data[i], data[i + 1]))
            ret.push_back(data[i]);

    }

    /**
     * @brief Handle last-point case 
     * @note At this point one can be sure that the data map has at least @c 3 elements
     */
    
    // Check whether the last point is close enough to the first-to-last to be clasified
    if(classified(data[data.size() - 1], data[data.size()])) {

        // Check if first-to-last point has been classified; if so, add last point to the return set
        if(classified(data[data.size() - 2], data[data.size() - 1]))
            ret.push_back(data[data.size()]);
        // Else, add both poitns to the return set
        else {
            ret.push_back(data[data.size() - 1]);
            ret.push_back(data[data.size()]);
        }

    }

    return ret;
}

/* ==================================================== RegionOfInterestFilter ==================================================== */

RegionOfInterestFilter::RegionOfInterestFilter(double roi_threshold) :
    roi_threshold { roi_threshold }
{ }


std::optional<poi_map_msgs::msg::Marker> RegionOfInterestFilter::transform(
    const poi_map_msgs::msg::Marker& incoming_marker,
    const std::vector<poi_map_msgs::msg::Marker>& all_incoming_markers
) {

    std::optional<poi_map_msgs::msg::Marker> ret;

    // If ROI is detectd
    if(not last_marker_in_roi and incoming_marker.score >= roi_threshold) {

        // Mark start of the ROI
        last_marker_in_roi = true;
        // Cache ROI's boundary
        last_roi_start_idx = processed_marker_idx;

    // Else, if end of ROI is detected
    } else if(
        last_marker_in_roi and (
            incoming_marker.score < roi_threshold or
            processed_marker_idx == all_incoming_markers.size()
        )
    ) {
        
        // Mark end of the ROI
        last_marker_in_roi = false;
        // Set center point of the ROI to be returned ( @note a + (b - a) / 2 = (a + b) / 2 )
        ret = all_incoming_markers[(processed_marker_idx + last_roi_start_idx) / 2];

    }

    // Increment index of the processed marker for the next iteration
    ++processed_marker_idx;

    return ret;
}


void RegionOfInterestFilter::reset() {
    processed_marker_idx = 0;
    last_marker_in_roi = false;
}

/* ====================================================== RectilinearMatcher ====================================================== */

RectilinearMatcher::RectilinearMatcher(const poi_map_msgs::msg::Point2D &threshold) :
    threshold { threshold }
{ }

bool RectilinearMatcher::match(
    const poi_map_msgs::msg::Marker& local_marker,
    const poi_map_msgs::msg::Marker& incoming_marker
) {
    return 
        std::abs(incoming_marker.position.x - local_marker.position.x) < threshold.x and
        std::abs(incoming_marker.position.y - local_marker.position.y) < threshold.y;
}

/* ===================================================== IntensityIncrementer ===================================================== */

IntensityIncrementer::IntensityIncrementer(double limit) :
    limit { limit }
{ }


double IntensityIncrementer::update(
    const poi_map_msgs::msg::Marker& local_marker,
    [[maybe_unused]] const poi_map_msgs::msg::Marker& incoming_marker
) {
    return (local_marker.score + 1 > limit) ? limit : local_marker.score + 1;
}

/* ========================================================= HalfawayMover ======================================================== */

poi_map_msgs::msg::Point2D HalfawayMover::update(
    const poi_map_msgs::msg::Marker& local_marker,
    const poi_map_msgs::msg::Marker& incoming_marker
) {

    poi_map_msgs::msg::Point2D ret;

    // Calculate the new position as a midpoint between given markers
    ret.x = ( local_marker.position.x + incoming_marker.position.x ) / 2.0;
    ret.y = ( local_marker.position.y + incoming_marker.position.y ) / 2.0;

    return ret;
}

/* ================================================================================================================================ */

} // End namespace velmwheel::points_of_interest
