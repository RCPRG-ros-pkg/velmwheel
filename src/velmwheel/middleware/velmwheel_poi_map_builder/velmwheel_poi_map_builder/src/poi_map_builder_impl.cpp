/* ============================================================================================================================ *//**
 * @file       poi_map_builder_impl.cpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 31st March 2022 9:53:56 pm
 * @modified   Thursday, 26th May 2022 12:07:29 am
 * @project    engineering-thesis
 * @brief      Definition of the ROS2 component node class building an intensity map of points of interest (POI)
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

/* =========================================================== Includes =========================================================== */

// System includes
#include <utility>
// TF includes
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
// Common includes
#include "node_common/node.hpp"
// Private includes
#include "velmwheel/poi_map_common.hpp"
#include "velmwheel/poi_map_builder_impl.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace velmwheel::points_of_interest {

/* ======================================================= Auxiliary methods ====================================================== */

/**
 * @brief Helper function constructing poi_map_msgs::msg::Point2D from 2-element
 *    std::array
 * 
 * @param array 
 *    array to be converted
 */
static inline poi_map_msgs::msg::Point2D make_point_2d(const std::array<double, 2> &array) {

    poi_map_msgs::msg::Point2D ret;

    // Fill the point
    ret.x = array[0];
    ret.y = array[1];

    return ret;
}

/* ======================================================== Auxiliary class ======================================================= */

std::vector<poi_map_msgs::msg::Marker> MapView::in_frame(const std::string &target_frame) {

    // Try to transform the map
    try {

        return transform_markers_map(
            tf_buffer,
            reference_frame,
            target_frame,
            stamp,
            map
        );

    // If failed to acquire the transformation, return
    } catch (std::exception &ex) {

        RCLCPP_WARN_STREAM(rclcpp::get_logger("MapView"), "Could not get transform from the actual to the target frame (" <<  ex.what() << ")");
        return std::vector<poi_map_msgs::msg::Marker>{ };
        
    }
    
}


MapView::operator std::vector<poi_map_msgs::msg::Marker>() {
    return map;
}


MapView::MapView(
    tf2_ros::Buffer &tf_buffer,
    const std::vector<poi_map_msgs::msg::Marker> &map,
    const std::string &reference_frame,
    const rclcpp::Time &stamp
) :
    tf_buffer { tf_buffer },
    map { map },
    reference_frame { reference_frame },
    stamp { stamp }
{ }

/* ========================================================== MapBuilderImpl ========================================================== */

MapBuilderConstructor MapBuilderImpl::make_constructor() {
    return MapBuilderConstructor {};
}


MapBuilderImpl::MapBuilderImpl(
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
) :
    // Initialize local configuration
    tf_buffer { tf_buffer },
    fixed_frame { fixed_frame },
    robot_frame { robot_frame },

    // Construct data filter
    data_filter { std::make_unique<DenseDataFilter>(distance_limit) },

    // Construct temporary-map builder
    temporary_map_builder { *poi_map::MapBuilder::make_constructor()
        .markers_limit(temporary_map_markers_limit)
        .transformation_strategy(std::make_unique<RegionOfInterestFilter>(intensity_threshold))
        .matching_strategy(std::make_unique<RectilinearMatcher>(make_point_2d(temporary_map_distance_matching_threshold_xy)))
        .score_update_strategy(std::make_unique<IntensityIncrementer>(intensity_limit))
        .position_update_strategy(std::make_unique<HalfawayMover>())
        .discarding_strategy(std::make_unique<poi_map::DiscardingStrategy>())
    },

    // Construct global-map builder
    map_builder { *poi_map::MapBuilder::make_constructor()
        .markers_limit(map_markers_limit)
        .transformation_strategy(std::make_unique<poi_map::TransformationStrategy>())
        .matching_strategy(std::make_unique<RectilinearMatcher>(make_point_2d(map_distance_matching_threshold_xy)))
        .score_update_strategy(std::make_unique<IntensityIncrementer>(intensity_limit))
        .position_update_strategy(std::make_unique<HalfawayMover>())
        .discarding_strategy(std::make_unique<poi_map::DiscardingStrategy>())
    },

    // Construct map-loader
    map_loader { save_intensity_threshold }
    
{ }


void MapBuilderImpl::update_map(const sensor_msgs::msg::PointCloud2 &msg) {

    // Convert message to intensity markers
    auto markers = convert(msg);
    // Filter input data
    markers = data_filter->filter(markers);
    // Convert filtered data to robot's frame of reference
    markers = transform_markers_map(
        tf_buffer,
        msg.header.frame_id,
        robot_frame,
        msg.header.stamp,
        markers
    );

    // Update 'temporary map'
    temporary_map_builder.update(markers);
    
    // Get reference to the 'temporary map'
    auto &temporary_map = temporary_map_builder.get_map();
    // Convert 'temporary map' markers to fixed's frame of reference
    markers = transform_markers_map(
        tf_buffer,
        robot_frame,
        fixed_frame,
        msg.header.stamp,
        temporary_map
    );
    
    // Update 'global map'
    map_builder.update(markers);   
}


void MapBuilderImpl::update_temporary_map(const sensor_msgs::msg::PointCloud2 &msg) {

    // Convert message to intensity markers
    auto markers = convert(msg);
    // Filter input data
    markers = data_filter->filter(markers);
    // Convert filtered data to robot's frame of reference
    markers = transform_markers_map(
        tf_buffer,
        msg.header.frame_id,
        robot_frame,
        msg.header.stamp,
        markers
    );

    // Update 'temporary map'
    temporary_map_builder.update(markers);
}

const std::string &MapBuilderImpl::get_fixed_frame() const {
    return fixed_frame;
}


const std::string &MapBuilderImpl::get_robot_frame() const {
    return robot_frame;
}


MapView MapBuilderImpl::get_temporary_map(const rclcpp::Time &stamp) const {
    return MapView {
        tf_buffer,
        temporary_map_builder.get_map(),
        robot_frame,
        stamp
    };
}


MapView MapBuilderImpl::get_map(const rclcpp::Time &stamp) const {
    return MapView {
        tf_buffer,
        map_builder.get_map(),
        fixed_frame,
        stamp
    };
}


void MapBuilderImpl::save_map(const std::string &filename) const {  
    map_loader.save_map(filename, map_builder.get_map());
}


void MapBuilderImpl::load_map(const std::string &filename) {  
    map_builder.set_map(map_loader.load_map(filename));
}

/* ==================================================== Auxiliary builder class =================================================== */

MapBuilderConstructor &MapBuilderConstructor::tf_buffer(tf2_ros::Buffer &buffer) {
    this->tf_buffer_ = &buffer;
    return *this;
}


MapBuilderConstructor &MapBuilderConstructor::distance_limit (double limit) {
    this->distance_limit_ = limit;
    return *this;
}


MapBuilderConstructor &MapBuilderConstructor::intensity_threshold (double threshold) {
    this->intensity_threshold_ = threshold;
    return *this;
}


MapBuilderConstructor &MapBuilderConstructor::fixed_frame (const std::string &frame) {
    this->fixed_frame_ = frame;
    return *this;
}


MapBuilderConstructor &MapBuilderConstructor::robot_frame (const std::string &frame) {
    this->robot_frame_ = frame;
    return *this;
}


MapBuilderConstructor &MapBuilderConstructor::intensity_limit (double limit) {
    this->intensity_limit_ = limit;
    return *this;
}


MapBuilderConstructor &MapBuilderConstructor::save_intensity_threshold (double threshold) {
    this->save_intensity_threshold_ = threshold;
    return *this;
}


MapBuilderConstructor &MapBuilderConstructor::map_markers_limit (std::size_t limit) {
    this->map_markers_limit_ = limit;
    return *this;
}


MapBuilderConstructor &MapBuilderConstructor::map_distance_matching_threshold_xy (const std::array<double, 2> &threshold) {
    this->map_distance_matching_threshold_xy_ = threshold;
    return *this;
}


MapBuilderConstructor &MapBuilderConstructor::temporary_map_markers_limit (std::size_t limit) {
    this->temporary_map_markers_limit_ = limit;
    return *this;
}


MapBuilderConstructor &MapBuilderConstructor::temporary_map_distance_matching_threshold_xy (const std::array<double, 2> &temporary_threshold) {
    this->temporary_map_distance_matching_threshold_xy_ = temporary_threshold;
    return *this;
}


MapBuilderImpl MapBuilderConstructor::operator*() const {

    // Check if all construction parameters has been set
    bool all_parameters_valid =
        (tf_buffer_ != nullptr)                                   and
        distance_limit_.has_value()                               and
        intensity_threshold_.has_value()                          and
        fixed_frame_.has_value()                                  and
        robot_frame_.has_value()                                  and
        intensity_limit_.has_value()                              and
        save_intensity_threshold_.has_value()                     and
        map_markers_limit_.has_value()                            and
        map_distance_matching_threshold_xy_.has_value()           and
        temporary_map_markers_limit_.has_value()                  and
        temporary_map_distance_matching_threshold_xy_.has_value();

    // If not all parameters has been set, throw error
    if(not all_parameters_valid)
        throw std::runtime_error{ "[MapBuilderConstructor] Cannot construct MapBuilderImpl with unset parameters" };

    // Construct the MapBuilderImpl
    return MapBuilderImpl {
        *tf_buffer_,
        *distance_limit_,
        *intensity_threshold_,
        *fixed_frame_,
        *robot_frame_,
        *intensity_limit_,
        *save_intensity_threshold_,
        *map_markers_limit_,
        *map_distance_matching_threshold_xy_,
        *temporary_map_markers_limit_,
        *temporary_map_distance_matching_threshold_xy_
    };
    
}

/* ================================================================================================================================ */

} // End namespace velmwheel::points_of_interest
