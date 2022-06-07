/* ============================================================================================================================ *//**
 * @file       global_localization_icp.cpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Monday, 4th April 2022 1:53:29 pm
 * @modified   Thursday, 26th May 2022 12:59:23 am
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
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/utils.h"
// PCL includes
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/registration/icp.h>
#include <pcl/common/io.h>
// Private includes
#include "velmwheel/global_localization_impl.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace velmwheel {

/* ================================================ Private methods (ICP algorithm) =============================================== */

std::optional<tf2::Transform> GlobalLocalizationImpl::localize_icp() {

    std::optional<tf2::Transform> robot_to_map_transform;

    // Don't try to localize if there are not at least two points in both maps
    if(global_map_markers->points.size() < 2 or local_map_markers->points.size() < 2)
        return robot_to_map_transform;

    using PointType = pcl::PointXYZ;
    using CloudType = pcl::PointCloud<PointType>;

    // Point cloud used to store markers from the 'local map' to be matched by ICP
    CloudType::Ptr markers_point_cloud{ new CloudType };
    // Point cloud used to store markers from the 'global map' to be matched by ICP
    CloudType::Ptr map_markers_point_cloud{ new CloudType };

    // Fill the 'local map' cloud 
    pcl::copyPointCloud(*global_map_markers, *markers_point_cloud);
    // Fill the 'global map' cloud  
    pcl::copyPointCloud(*local_map_markers, *map_markers_point_cloud);

    pcl::IterativeClosestPoint<PointType, PointType> icp;
    
    /**
     * @brief Preconfigure the ICP
     * @note The aim of the function is to find transformation <b>from 'robot'</b>
     *    (i.e. from the cloud of 'local map' markers) <b>to 'map'</b> (i.e. to
     *    the cloud of 'global map' markers). For this reason cloud of 'local map'
     *    markers is set as a source cloud
     */
    
    // Set source and target clouds
    icp.setInputSource(markers_point_cloud);
    icp.setInputTarget(map_markers_point_cloud);
    // Set algorithm configuration
    icp.setMaxCorrespondenceDistance(icp_config.max_correspondance_distance);  
    icp.setTransformationEpsilon(icp_config.transformation_epsilon); 
    icp.setEuclideanFitnessEpsilon(icp_config.euclidean_fitness_epsilon); 
    icp.setMaximumIterations(static_cast<int>(icp_config.maximum_iterations)); 

    // Run the algorithm
    CloudType Final;
    icp.align(Final);

    // If algorithm didn't manage to converge, return failure
    if(not icp.hasConverged())
        return robot_to_map_transform;

    // Otherwise parse resulting transformation
    auto transformation = icp.getFinalTransformation();
    // Parse resulting 3D rotation in the tf2::Matrix
    tf2::Matrix3x3 rotation_3d(
        transformation(0, 0), transformation(0, 1), transformation(0, 2),
        transformation(1, 0), transformation(1, 1), transformation(1, 2),
        transformation(2, 0), transformation(2, 1), transformation(2, 2)
    );
    
    // Transform 3D rotation matrix into 2D rotation quaternion (i.e. keep only Yaw rotation along Z axis)
    tf2::Quaternion rotation_2d;
    rotation_3d.getRotation(rotation_2d);
    rotation_2d.setRPY(0.0, 0.0, tf2::getYaw(rotation_2d));
    // Parse resulting 2D translation in the tf2::Vector
    tf2::Vector3 translation_2d(
        transformation(0, 3),
        transformation(1, 3),
        0.0
    );

    // Set resulting tf2::Transformation
    robot_to_map_transform->setRotation(rotation_2d);
    robot_to_map_transform->setOrigin(translation_2d);

    // Return result
    return robot_to_map_transform;
}

/* ================================================================================================================================ */

} // End namespace velmwheel 
