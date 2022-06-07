/* ============================================================================================================================ *//**
 * @file       laser_scan_converter.cpp
 * @author     Ivan Dryanovski
 * @author     William Morris
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 19th May 2022 9:53:09 am
 * @modified   Wednesday, 25th May 2022 11:08:14 pm
 * @project    engineering-thesis
 * @brief      Definitions of the ROS2 node class converting sensor_msgs::msg::LaserScan data into the PCL-specific PointCloud
 *             representation
 * 
 * @copyright (c) 2010, 2011, Ivan Dryanovski, William Morris
 * 
 *    Copyright (c) 2010, 2011, Ivan Dryanovski, William Morris
 *    All rights reserved.
 *    
 *    Redistribution and use in source and binary forms, with or without
 *    modification, are permitted provided that the following conditions are met:
 *    
 *        * Redistributions of source code must retain the above copyright
 *          notice, this list of conditions and the following disclaimer.
 *        * Redistributions in binary form must reproduce the above copyright
 *          notice, this list of conditions and the following disclaimer in the
 *          documentation and/or other materials provided with the distribution.
 *        * Neither the name of the CCNY Robotics Lab nor the names of its
 *          contributors may be used to endorse or promote products derived from
 *          this software without specific prior written permission.
 *    
 *    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *    ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *    LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *    CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *    SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *    INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *    CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *    ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *    POSSIBILITY OF SUCH DAMAGE.
 * 
 */// ============================================================================================================================= */

/* =========================================================== Includes =========================================================== */

// System includes
#include <limits>
// Common includes
#include "node_common/node.hpp"
// Private includes
#include "scan_tools/laser_scan_converter.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace scan_tools {

/* ========================================================= Ctors & dtors ======================================================== */

LaserScanConverter::LaserScanConverter(const rclcpp::NodeOptions & options) : 
    rclcpp::Node("laser_scan_converter", options)
{

    // Initialize subscriber used to acquire input laser scans
    *node_common::communication::make_subscriber_builder(scan_sub)
        .node(*this)
        .name(SCAN_SUB_TOPIC_NAME)
        .qos(TOPIC_QUEUE_SIZE)
        .callback(*this, &LaserScanConverter::scan_callback);

    // Initialize publisher used to provide output point cloud
    *node_common::communication::make_publisher_builder(cloud_pub)
        .node(*this)
        .name(CLOUD_PUB_TOPIC_NAME)
        .qos(TOPIC_QUEUE_SIZE);

    node_common::node::print_hello(*this);
}

LaserScanConverter::~LaserScanConverter() {
    node_common::node::print_goodbye(*this);
}

/* =========================================================== Callbacks ========================================================== */

void LaserScanConverter::scan_callback(const sensor_msgs::msg::LaserScan &scan_msg) {

    // Allocate the target cloud
    CloudType cloud;

    // Allocated chunk of memory for points
    cloud.points.reserve(scan_msg.ranges.size());
    
    // Assume that all measurements will be valid
    cloud.is_dense = true;

    // Iterate over data points in the laser scan
    for (unsigned int i = 0; i < scan_msg.ranges.size(); ++i) {

        // Get measured range to the point
        auto range = scan_msg.ranges[i];

        // If the measured range is valid, parse it into the cloud
        if (range > scan_msg.range_min && range < scan_msg.range_max) {

            // Get angle of the current measurement
            float angle = scan_msg.angle_min + i * scan_msg.angle_increment;
            // Emplace the new point in the cloud using transformation from polar to cartesian coordinates system
            cloud.points.emplace_back();
            cloud.points.back().x         = range * std::cos(angle); // X component 
            cloud.points.back().y         = range * std::sin(angle); // Y component 
            cloud.points.back().z         = 0.0;                     // Z component 
            cloud.points.back().intensity = scan_msg.intensities[i]; // Intensity component 

        // If the measured range is invalid, insert invalid point instead
        } else {

            // If at least a single measurement is invalid, mark cloud as parse
            cloud.is_dense = true;
        
            // Emplace invalid point
            cloud.points.emplace_back();
            cloud.points.back().x         = std::numeric_limits<float>::quiet_NaN(); // X component 
            cloud.points.back().y         = std::numeric_limits<float>::quiet_NaN(); // Y component 
            cloud.points.back().z         = std::numeric_limits<float>::quiet_NaN(); // Z component 
            cloud.points.back().intensity = std::numeric_limits<float>::quiet_NaN(); // Intensity component 
            
        }
    }

    // Set dimensionality of cloud's data
    cloud.width = scan_msg.ranges.size();
    cloud.height = 1;

    // Create output ROS message
    sensor_msgs::msg::PointCloud2 cloud_msg;

    // Convert cloud to message
    cloud_to_message(cloud, cloud_msg);
    // Fille message's header
    cloud_msg.header.frame_id = scan_msg.header.frame_id;
    cloud_msg.header.stamp = scan_msg.header.stamp;
    
    // Publish the cloud
    cloud_pub->publish(cloud_msg);

}

/* ================================================================================================================================ */

} // End namespace scan_tools

/* ======================================================== Nodes' registry ======================================================= */

RCLCPP_COMPONENTS_REGISTER_NODE(scan_tools::LaserScanConverter)

/* ================================================================================================================================ */

