/* ============================================================================================================================ *//**
 * @file       poi_map_common.cpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Sunday, 3rd April 2022 2:01:01 am
 * @modified   Thursday, 26th May 2022 12:13:43 am
 * @project    engineering-thesis
 * @brief      Set of common utilities used across the package
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

/* =========================================================== Includes =========================================================== */

// PCL includes
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
// Private includes
#include "velmwheel/poi_map_common.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace velmwheel::points_of_interest {

/* ======================================================== Helper methods ======================================================== */

std::vector<poi_map_msgs::msg::Marker> convert(const sensor_msgs::msg::PointCloud2 &data) {

    pcl::PointCloud<pcl::PointXYZI> cloud;

    // Convert message to cloud
    pcl::fromROSMsg(data, cloud);

    std::vector<poi_map_msgs::msg::Marker> ret;

    //Reserve memory for markers
    ret.reserve(cloud.points.size());
    // Parse cloud points into the map of markers
    for(auto &point : cloud) {

        // Add marker to the output vector
        ret.emplace_back();
        // Fill the added marker
        ret.back().position.x = point.x;
        ret.back().position.y = point.y;
        ret.back().score      = point.intensity;

    }

    return ret;
}


std::vector<poi_map_msgs::msg::Marker> transform_markers_map(
    tf2_ros::Buffer &tf_buffer,
    const std::string &reference_frame,
    const std::string &target_frame,
    const rclcpp::Time &stamp,
    const std::vector<poi_map_msgs::msg::Marker> &markers
) {

    static constexpr double TRANSFORM_WAITING_TIMEOUT_S = 1.0;
    
    // If target and actual reference frame match, return the map
    if(target_frame == reference_frame)
        return markers;

    geometry_msgs::msg::TransformStamped reference_to_target_transform_msg;

    // Wait for the transformation to be available
    reference_to_target_transform_msg = tf_buffer.lookupTransform(
        target_frame,
        reference_frame,
        stamp,
        rclcpp::Duration::from_seconds( TRANSFORM_WAITING_TIMEOUT_S )
    );

    std::vector<poi_map_msgs::msg::Marker> ret;
    
    // On success, allocate memory for output map
    ret.reserve(markers.size());

    tf2::Transform reference_to_target_transform;

    // Convert transform message to tf2::Transform
    tf2::fromMsg(reference_to_target_transform_msg.transform, reference_to_target_transform);

    // Transform markers' positions into the fixed frame of reference
    for(auto &marker : markers) {

        // Parse marker's position into the TF vector
        tf2::Vector3 position {
            marker.position.x,
            marker.position.y,
            0.0
        };

        // Transform position into the fixed frame of reference
        position = reference_to_target_transform(position);
        // Add point to the output
        ret.emplace_back();
        // Set marker's position in the new frame of reference
        ret.back().position.x = position.x();
        ret.back().position.y = position.y();
        ret.back().score      = marker.score;
        
    }

    return ret;
}

/* ================================================================================================================================ */

} // End namespace velmwheel::points_of_interest
