/* ============================================================================================================================ *//**
 * @file       poi_map_common.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Sunday, 3rd April 2022 2:01:01 am
 * @modified   Thursday, 19th May 2022 11:18:23 am
 * @project    engineering-thesis
 * @brief      Set of common utilities used across the package
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __VELMWHEEL_POI_MAP_COMMON_H__
#define __VELMWHEEL_POI_MAP_COMMON_H__

/* =========================================================== Includes =========================================================== */

// ROS includes
#include "rclcpp/time.hpp"
// Interfaces includes
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "poi_map_msgs/msg/markers_map.hpp"
// TF includes
#include "tf2_ros/buffer.h"

/* ========================================================== Namespaces ========================================================== */

namespace velmwheel::points_of_interest {

/* ======================================================== Helper methods ======================================================== */

/**
 * @brief Converts incoming @p data in form of point cloud into the list of intensity markers
 * 
 * @param data 
 *    data to be converted
 * @returns 
 *    @p data converted to list of intensity markers
 */
std::vector<poi_map_msgs::msg::Marker> convert(const sensor_msgs::msg::PointCloud2 &data);

/**
 * @brief Transforms map of intensity markers from the @p reference_frame to the
 *    @p target_frame obtaining suitable transformation via @p tf_buffer
 * 
 * @param tf_buffer 
 *    buffer utilized to obtain suitable transformation
 * @param reference_frame 
 *    refernce frame of the @p markers map
 * @param target_frame 
 *    target reference frame
 * @param stamp 
 *    timestamp of the target transformation
 * @param markers 
 *    markers to be transformed
 * @returns 
 *    copy of @p markers transformed to the @p target_frame reference frame
 * 
 * @throws tf2::TransformException
 *    if the target transformation could not be obtained
 */
std::vector<poi_map_msgs::msg::Marker> transform_markers_map(
    tf2_ros::Buffer &tf_buffer,
    const std::string &reference_frame,
    const std::string &target_frame,
    const rclcpp::Time &stamp,
    const std::vector<poi_map_msgs::msg::Marker> &markers
);

/* ================================================================================================================================ */

} // End namespace velmwheel::points_of_interest

#endif
