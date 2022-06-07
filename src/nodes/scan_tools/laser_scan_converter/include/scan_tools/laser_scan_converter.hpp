/* ============================================================================================================================ *//**
 * @file       laser_scan_converter.hpp
 * @author     Ivan Dryanovski
 * @author     William Morris
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 19th May 2022 9:53:09 am
 * @modified   Wednesday, 25th May 2022 11:07:59 pm
 * @project    engineering-thesis
 * @brief      Declaration of the ROS2 node class converting sensor_msgs::msg::LaserScan data into the PCL-specific PointCloud
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

#ifndef __SCAN_TOOLS_LASER_SCAN_CONVERTER_H__
#define __SCAN_TOOLS_LASER_SCAN_CONVERTER_H__

/* =========================================================== Includes =========================================================== */

// ROS includes
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rclcpp_components/register_node_macro.hpp"
// Message includes
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
// PCL includes
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
// Private includes
#include "node_common/communication.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace scan_tools {

/* ============================================================= Node ============================================================= */

/**
 * @brief ROS2 node class converting sensor_msgs::msg::LaserScan data into the PCL-specific PointCloud
 *    representation
 */
class RCLCPP_PUBLIC LaserScanConverter: public rclcpp::Node {

public: /* ------------------------------------------------- Topics's parameters -------------------------------------------------- */

    /// Size of the topics' queue
    static constexpr std::size_t TOPIC_QUEUE_SIZE = 1000;

    /// Name of the input laser scan topic
    static constexpr auto SCAN_SUB_TOPIC_NAME = "scan";
    
    /// Name of the output point cloud topic 
    static constexpr auto CLOUD_PUB_TOPIC_NAME = "cloud";

public: /* ------------------------------------------------- Public ctors & dtors ------------------------------------------------- */

    /**
     * @brief Construct a new Laser Scan Converter object
     * 
     * @param options 
     *    configuration of the node
     */
    LaserScanConverter(const rclcpp::NodeOptions & options);

    /**
     * @brief Destroy the Laser Scan Converter object logging goodbye message 
     *    to the rosout
     */
    ~LaserScanConverter();

private: /* ------------------------------------------------- Point clouds typing ------------------------------------------------- */

    /// Point type used when interacting with PCL library
    using PointType = pcl::PointXYZI;
    /// Cloud type used when interacting with PCL library
    using CloudType = pcl::PointCloud<PointType>;

    /// Alias for cloud-to-message conversion function
    static inline void cloud_to_message(const CloudType &cloud, sensor_msgs::msg::PointCloud2 &cloud_msg) {
        pcl::toROSMsg(cloud, cloud_msg);
    }
    
private: /* ------------------------------------------------------ Callbacks ------------------------------------------------------ */

    /**
     * @brief Callback for the incoming slaser scan messages
     * @param scan_msg 
     *    incoming message
     */
    void scan_callback(const sensor_msgs::msg::LaserScan &scan_msg);

private: /* ---------------------------------------------------- ROS interfaces --------------------------------------------------- */

	/// Subscriber interface used to acquire input laser scans
	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
	/// Publisher interface used to provide output point cloud
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub;
    
};

/* ================================================================================================================================ */

} // End namespace scan_tools

#endif
