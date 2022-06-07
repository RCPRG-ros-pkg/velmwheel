/* ============================================================================================================================ *//**
 * @file       global_localization_impl.cpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Monday, 4th April 2022 1:53:29 pm
 * @modified   Thursday, 26th May 2022 2:43:33 am
 * @project    engineering-thesis
 * @brief      Definitions fo the GlobalLocalizationImpl class implementing set of algorithms utilized by the GlobalLocalization ROS node
 *             class
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

/* =========================================================== Includes =========================================================== */

// System includes
#include <algorithm>
#include <cmath>
// TF includes
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/utils.h"
// Eigen includes
#include <Eigen/Core>
#include <Eigen/SVD>
// PCL includes
#include <pcl/point_types.h>
#include "pcl/registration/icp.h"
#include "pcl/registration/correspondence_estimation.h"
// Private includes
#include "velmwheel/global_localization_impl.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace velmwheel {

/* ========================================================= Public ctors ========================================================= */

GlobalLocalizationImpl::GlobalLocalizationImpl(
    const ConfigSTD &std_config,
    const ConfigICP &icp_config
) :
    std_config { std_config },
    icp_config { icp_config }
{

    // Verify whether valid configuration of STD matching methods has been given
    bool invalid_config = 
        std_config.matching_method          == ConfigSTD::MatchingMethod::PreviousTransform and
        std_config.fallback_matching_method == ConfigSTD::MatchingMethod::PreviousTransform;

    // If invalid config given, force default config
    if(invalid_config) {
        const_cast<ConfigSTD::MatchingMethod &>(std_config.fallback_matching_method) =
            ConfigSTD::MatchingMethod::NeighboursDistance;
    }

}

/* ========================================================== Public API ========================================================== */

void GlobalLocalizationImpl::update_odometry(const nav_msgs::msg::Odometry &msg) {

    // Cache inversion of the previous odometry-based transformation
    odom_to_robot_transform_previous = robot_to_odom_transform.inverse();

    /**
     * @note In the following coversions both Z and ɑ/β (roll and pitch angles) components 
     *    of the transformation are set to 0.0. This is because the localization algorith
     *    operates in the 2D space
     */

    // Calculate current translation form 'robot' to 'odom' frame based on odometry
    robot_to_odom_transform.setOrigin(
        tf2::Vector3{
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            0.0
        }
    );

    // Calculate current rotation form 'robot' to 'odom' frame based on odometry
    tf2::Quaternion q;
    tf2::fromMsg(msg.pose.pose.orientation, q);
    // Remove rotation along both X and Y axis
    q.setRPY(0.0, 0.0, tf2::getYaw(q));
    // Cache the rotation
    robot_to_odom_transform.setRotation(q);

}


void GlobalLocalizationImpl::update_scan(const poi_map_msgs::msg::MarkersStamped &msg) {

    // Resize the local copy of map
    local_map_markers->resize(msg.markers.size());
    // Copy content
    for(unsigned i = 0; i < msg.markers.size(); ++i) {
        local_map_markers->points[i].x = msg.markers[i].position.x;
        local_map_markers->points[i].y = msg.markers[i].position.y;
    }
    
}


void GlobalLocalizationImpl::update_map(const poi_map_msgs::msg::MarkersMap &msg) {
    
    // Resize the local copy of map
    global_map_markers->resize(msg.markers.size());
    // Copy content
    for(unsigned i = 0; i < msg.markers.size(); ++i) {
        global_map_markers->points[i].x = msg.markers[i].position.x;
        global_map_markers->points[i].y = msg.markers[i].position.y;
    }
    
}


std::optional<tf2::Transform> GlobalLocalizationImpl::estimate_odom_pose_std() {
    return estimate_odom_pose( localize_std() );
}


std::optional<tf2::Transform> GlobalLocalizationImpl::estimate_odom_pose_icp() {
    return estimate_odom_pose( localize_icp() );
}

/* =================================================== Private methods (common) =================================================== */

std::optional<tf2::Transform> GlobalLocalizationImpl::estimate_odom_pose(const std::optional<tf2::Transform> &robot_to_map_transform) const {

    std::optional<tf2::Transform> odom_to_map_transform;

    // On failure return an empty value
    if(not robot_to_map_transform.has_value())
        return odom_to_map_transform;

    /**
     * @brief On success calculate estimated transformation from 'odom' to 'map' (i.e. pose
     *   of the 'odom' frame in the 'map' frame of reference) based on two transformations
     * 
     *      - from 'odom' to 'robot' (obtained from odometry data)
     *      - from 'robot' to 'map' (obtained from PoI matching algorithm)
     * 
     */
    odom_to_map_transform = (*robot_to_map_transform) * odom_to_robot_transform_previous;

    // Return estimation result
    return odom_to_map_transform;
}

/* ================================================================================================================================ */

} // End namespace velmwheel 
