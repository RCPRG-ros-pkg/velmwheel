/* ============================================================================================================================ *//**
 * @file       global_localization_utils.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 6th April 2022 7:48:15 pm
 * @modified   Thursday, 7th April 2022 3:11:03 am
 * @project    engineering-thesis
 * @brief      Set of abstract utilities used across the package
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __VELMWHEEL_GLOBAL_LOCALIZATION_IMPL_UTILS_H__
#define __VELMWHEEL_GLOBAL_LOCALIZATION_IMPL_UTILS_H__

/* =========================================================== Includes =========================================================== */

#include "velmwheel/global_localization_utils.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace velmwheel {
namespace utils {

/* ======================================================= Helper functions ======================================================= */

pcl::PointXY to_2d(const pcl::PointXYZ &p) {
    return pcl::PointXY{ p.x, p.y };
}


pcl::PointCloud<pcl::PointXY>::Ptr to_2d(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pc) {

    pcl::PointCloud<pcl::PointXY>::Ptr ret { new pcl::PointCloud<pcl::PointXY>( pc->size(), 1 ) };

    // Fill the cloud
    for(unsigned i = 0; i < pc->size(); ++i)
        ret->points[i] = to_2d(pc->points[i]);

    return ret;
}


pcl::PointXYZ to_3d(const pcl::PointXY &p) {
    return pcl::PointXYZ{ p.x, p.y, 0.0 };
}


pcl::PointCloud<pcl::PointXYZ>::Ptr to_3d(const pcl::PointCloud<pcl::PointXY>::Ptr &pc) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr ret { new pcl::PointCloud<pcl::PointXYZ>( pc->size(), 1 ) };

    // Fill the cloud
    for(unsigned i = 0; i < pc->size(); ++i)
        ret->points[i] = to_3d(pc->points[i]);

    return ret;
}


template<typename PoinT>
float distance(const PoinT &p, const PoinT &q) {
    return pcl::geometry::distance(p, q);
};


float distance(const pcl::PointXY &p, const pcl::PointXYZ &q) {
    return distance(p, to_2d(q));
};


float distance(const pcl::PointXYZ &p, const pcl::PointXY &q) {
    return distance(q, p);
};


template<typename PoinT>
float distance2(const PoinT &p, const PoinT &q) {
    return pcl::geometry::squaredDistance(p, q);
};


float distance2(const pcl::PointXY &p, const pcl::PointXYZ &q) {
    return distance2(p, to_2d(q));
};


float distance2(const pcl::PointXYZ &p, const pcl::PointXY &q) {
    return distance2(q, p);
};


pcl::PointXY translation(const pcl::PointXY &p, const pcl::PointXY &q) {
    return pcl::PointXY(q.x - p.x, q.y - p.y);
}


Eigen::Affine3d tf_to_eigen(const tf2::Transform &tf) {

    Eigen::Affine3d eigen;

    // Convert tf2 transformation to transform msg
    auto msg = tf2::toMsg(tf);
    // Convert msg transformation to eigen
    eigen = tf2::transformToEigen(msg);

    return eigen;
}


Eigen::Affine3d tf_to_eigen_2d(const tf2::Transform &tf) {

    Eigen::Affine3d eigen;

    // Convert tf2 translation to Eigen
    eigen.translation() <<
        tf.getOrigin().x(),
        tf.getOrigin().y(),
        0.0;
    // Convert tf2 rotation to Eigen
    eigen.rotate(Eigen::AngleAxisf(
        tf2::getYaw(tf.getRotation()),
        Eigen::Vector3f::UnitZ()
    ));

    return eigen;
}

/* ================================================================================================================================ */

} // End namespace utils
} // End namespace velmwheel

#endif
