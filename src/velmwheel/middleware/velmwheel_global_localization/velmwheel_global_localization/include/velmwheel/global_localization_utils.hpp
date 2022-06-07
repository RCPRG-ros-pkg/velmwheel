/* ============================================================================================================================ *//**
 * @file       global_localization_utils.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 6th April 2022 7:48:15 pm
 * @modified   Thursday, 7th April 2022 3:10:17 am
 * @project    engineering-thesis
 * @brief      Set of abstract utilities used across the package
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __VELMWHEEL_GLOBAL_LOCALIZATION_UTILS_H__
#define __VELMWHEEL_GLOBAL_LOCALIZATION_UTILS_H__

/* =========================================================== Includes =========================================================== */

// System includes
#include <algorithm>
#include <vector>
// TF includes
#include "tf2/LinearMath/Transform.h"
#include "tf2/utils.h"
#include "tf2_eigen/tf2_eigen.h"
// Eigen includes
#include <Eigen/Core>
// PCL includes
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

/* ========================================================== Namespaces ========================================================== */

namespace velmwheel {
namespace utils {

/* ======================================================= Helper functions ======================================================= */

/**
 * @brief Helper function projecting 3D point on the Z=0 plane
 * 
 * @param p 
 *    point to be projected
 */
inline pcl::PointXY to_2d(const pcl::PointXYZ &p);

/**
 * @brief Helper function projecting 3D point cloud on the Z=0 plane
 * 
 * @param pc
 *    point cloud to be projected
 */
inline pcl::PointCloud<pcl::PointXY>::Ptr to_2d(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pc);


/**
 * @brief Helper function projecting 2D point to the 3D space
 * 
 * @param p 
 *    point to be projected
 */
inline pcl::PointXYZ to_3d(const pcl::PointXY &p);


/**
 * @brief Helper function projecting 2D point cloud to the 3D space
 * 
 * @param pc
 *    point cloud to be projected
 */
inline pcl::PointCloud<pcl::PointXYZ>::Ptr to_3d(const pcl::PointCloud<pcl::PointXY>::Ptr &pc);


/**
 * @brief Helper function calculating Euclidean distance between two 2D/3D points
 * 
 * @param p 
 *    the first point
 * @param q 
 *    the second point
 */
template<typename PoinT>
inline float distance(const PoinT &p, const PoinT &q);


/**
 * @brief Helper function calculating Euclidean distance between 3D projected
 *    on the Z=0 plane and the 2D point
 * 
 * @param p 
 *    the first point
 * @param q 
 *    the second point
 */
inline float distance(const pcl::PointXY &p, const pcl::PointXYZ &q);


/**
 * @brief Helper function calculating Euclidean distance between 3D projected
 *    on the Z=0 plane and the 2D point
 * 
 * @param p 
 *    the first point
 * @param q 
 *    the second point
 */
inline float distance(const pcl::PointXYZ &p, const pcl::PointXY &q);


/**
 * @brief Helper function calculating translation from point @p p to point @p q
 */
inline pcl::PointXY translation(const pcl::PointXY &p, const pcl::PointXY &q);

/**
 * @brief Helper function calculating squared Euclidean distance between two 2D/3D points
 * 
 * @param p 
 *    the first point
 * @param q 
 *    the second point
 */
template<typename PoinT>
inline float distance2(const PoinT &p, const PoinT &q);


/**
 * @brief Helper function calculating squared Euclidean distance between 3D projected
 *    on the Z=0 plane and the 2D point
 * 
 * @param p 
 *    the first point
 * @param q 
 *    the second point
 */
inline float distance2(const pcl::PointXY &p, const pcl::PointXYZ &q);


/**
 * @brief Helper function calculating squared Euclidean distance between 3D projected
 *    on the Z=0 plane and the 2D point
 * 
 * @param p 
 *    the first point
 * @param q 
 *    the second point
 */
inline float distance2(const pcl::PointXYZ &p, const pcl::PointXY &q);

/**
 * @brief Transforms TF2 transformation into the Eigen::Affine3d transformation
 * 
 * @param tf 
 *    transformation to be converted
 * @returns 
 *    @p tf converted to Eigen::Affine3d transformation
 */
inline Eigen::Affine3d tf_to_eigen(const tf2::Transform &tf);

/**
 * @brief Transforms TF2 transformation into the Eigen::Affine3d transformation passing
 *    only 2D parameters of the transformation (X/Y translation and θ rotation)
 * 
 * @param tf 
 *    transformation to be converted
 * @returns 
 *    @p tf converted to Eigen::Affine3d transformation
 */
inline Eigen::Affine3d tf_to_eigen_2d(const tf2::Transform &tf);

/* ================================================================================================================================ */

} // End namespace utils
} // End namespace velmwheel

/* ==================================================== Implementation includes =================================================== */

#include "velmwheel/impl/global_localization_utils.hpp"

/* ================================================================================================================================ */

#endif
