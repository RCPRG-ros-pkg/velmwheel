/* ============================================================================================================================ *//**
 * @file       global_localization_std.cpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Monday, 4th April 2022 1:53:29 pm
 * @modified   Thursday, 26th May 2022 2:09:41 am
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
#include "tf2_eigen/tf2_eigen.h"
// Eigen includes
#include <Eigen/Core>
#include <Eigen/SVD>
// PCL includes
#include <pcl/common/geometry.h>
#include <pcl/point_types.h>
#include "pcl/registration/icp.h"
#include "pcl/registration/correspondence_estimation.h"
// Private includes
#include "velmwheel/global_localization_impl.hpp"
#include "velmwheel/global_localization_algo.hpp"
#include "velmwheel/global_localization_utils.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace velmwheel {

/* ========================================================= Helper types ========================================================= */

/**
 * @brief Description of the distance between two points of given indeces
 */
struct DistanceDescriptor {

    /// Index of the local-map point
    std::size_t local_idx;
    /// Index of the global-map point
    std::size_t global_idx;
    /// Distance between points
    float distance;
    
};

/* ======================================================= Helper functions ======================================================= */

/**
 * @brief Helper function projecting 3D point on the Z=0 plane
 */
pcl::PointXY to2d(const pcl::PointXYZ &p) {
    return pcl::PointXY{ p.x, p.y };
}

/**
 * @brief Helper function projecting 3D point cloud on the Z=0 plane
 */
pcl::PointCloud<pcl::PointXY>::Ptr to2d(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pc) {

    pcl::PointCloud<pcl::PointXY>::Ptr ret { new pcl::PointCloud<pcl::PointXY>( pc->size(), 1 ) };

    // Fill the cloud
    for(unsigned i = 0; i < pc->size(); ++i)
        ret->points[i] = to2d(pc->points[i]);

    return ret;
}

/**
 * @brief Helper function projecting 2D point to the 3D space
 */
pcl::PointXYZ to3d(const pcl::PointXY &p) {
    return pcl::PointXYZ{ p.x, p.y, 0.0 };
}

/**
 * @brief Helper function projecting 2D point cloud to the 3D space
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr to3d(const pcl::PointCloud<pcl::PointXY>::Ptr &pc) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr ret { new pcl::PointCloud<pcl::PointXYZ>( pc->size(), 1 ) };

    // Fill the cloud
    for(unsigned i = 0; i < pc->size(); ++i)
        ret->points[i] = to3d(pc->points[i]);

    return ret;
}

/**
 * @brief Helper function calculating Euclidean distance between two 2D/3D points
 */
template<typename PoinT>
float distance(const PoinT &p, const PoinT &q) {
    return pcl::geometry::distance(p, q);
};

/**
 * @brief Helper function calculating Euclidean distance between 3D projected
 *    on the Z=0 plane and the 2D point
 */
float distance(const pcl::PointXY &p, const pcl::PointXYZ &q) {
    return distance(p, to2d(q));
};

/**
 * @brief Helper function calculating Euclidean distance between 3D projected
 *    on the Z=0 plane and the 2D point
 */
float distance(const pcl::PointXYZ &p, const pcl::PointXY &q) {
    return distance(q, p);
};

/**
 * @brief Helper function calculating squared Euclidean distance between two 2D/3D points
 */
template<typename PoinT>
float distance2(const PoinT &p, const PoinT &q) {
    return pcl::geometry::squaredDistance(p, q);
};

/**
 * @brief Helper function calculating squared Euclidean distance between 3D projected
 *    on the Z=0 plane and the 2D point
 */
float distance2(const pcl::PointXY &p, const pcl::PointXYZ &q) {
    return distance2(p, to2d(q));
};

/**
 * @brief Helper function calculating squared Euclidean distance between 3D projected
 *    on the Z=0 plane and the 2D point
 */
float distance2(const pcl::PointXYZ &p, const pcl::PointXY &q) {
    return distance2(q, p);
};

/* ===================================================== Private (localization) =================================================== */

std::optional<tf2::Transform> GlobalLocalizationImpl::localize_std() {

    std::optional<tf2::Transform> robot_to_map_transform;

    // Don't try to localize if there are not at least two points in both maps
    if(global_map_markers->points.size() < 2 or local_map_markers->points.size() < 2)
        return robot_to_map_transform;

    /* -------------------------------- Match markers -------------------------------- */

    std::vector<CorrsepondingPoints> corresponding_points;

    // By default try to use standard algorithm
    corresponding_points = match_markers(std_config.matching_method);
    // On failure, fall-back to algorithm
    if(corresponding_points.size() < 2)
        corresponding_points = match_markers(std_config.fallback_matching_method);

    /* -------------------------------- Register maps -------------------------------- */
    
    // If markers form both maps has been succesfully matched, estimate robot's pose in the 'map' frame
    if(corresponding_points.size() > 2) 
        robot_to_map_transform = estimate_robot_pose(corresponding_points);

    // Cache resulting transform to be used for markers-matching in the next iteration
    if(robot_to_map_transform.has_value())
        last_robot_to_map_transform = robot_to_map_transform;

    // Return result
    return robot_to_map_transform;
}

/* =============================================== Private methods (markers-matching) ============================================= */

std::vector<GlobalLocalizationImpl::CorrsepondingPoints> GlobalLocalizationImpl::match_markers(
    ConfigSTD::MatchingMethod match_type
) const {

    std::vector<CorrsepondingPoints> corresponding_points;

    // By default try to use standard algorithm
    switch(match_type) {
        case ConfigSTD::MatchingMethod::PreviousTransform:
            corresponding_points = match_markers_with_previous_transform(); break;
        case ConfigSTD::MatchingMethod::NeighboursDistance:
            corresponding_points = match_markers_with_neighbours_distance(); break;
        case ConfigSTD::MatchingMethod::NeighboursTranslation:
            corresponding_points = match_markers_with_neighbours_translation(); break;
        default:
            break;
    }
    
    return corresponding_points;
}


std::vector<GlobalLocalizationImpl::CorrsepondingPoints> GlobalLocalizationImpl::match_markers_with_previous_transform() const {

    std::vector<CorrsepondingPoints> corresponding_points;

    /**
     * On the first iteration fail due to lacking transformation from 'robot' to 'map' 
     * FoR (calling function shall fall-back to aproximate matching)
     */
    if(not last_robot_to_map_transform.has_value())
        return corresponding_points;

    // Convert transformation to PCL-compliant representation
    Eigen::Affine3d last_robot_to_map_eigen_transform = utils::tf_to_eigen(*last_robot_to_map_transform);

    // Match points
    algorithm::match_overlapping_unordered(
        
        // Source/target markers
        local_map_markers->points,
        global_map_markers->points,

        // Metric
        [](const pcl::PointXY &p, const pcl::PointXY &q) { return utils::distance2(p, q); },

        // Accumulator
        [&corresponding_points, this](const algorithm::Correspondance<float> &match) { 
            corresponding_points.emplace_back( CorrsepondingPoints {
                .local  = local_map_markers->points[match.source_idx],
                .global = global_map_markers->points[match.target_idx]
            });
        },

        // Materic limit for matched points (squared, as the 'squared distane' metric is used)
        std::pow(std_config.previous_transform_match_threshold, 2),

        // Source markers transformation
        [&last_robot_to_map_eigen_transform](const pcl::PointXY p) {
            return utils::to_2d( pcl::transformPoint(to3d(p), last_robot_to_map_eigen_transform) );
        }
        
    );

    return corresponding_points;
}



std::vector<GlobalLocalizationImpl::CorrsepondingPoints> GlobalLocalizationImpl::match_markers_with_neighbours_distance() const {
    return match_markers_with_neighbours_metric(
        
        /**
         * Intermediate callback calculating feature of pair <local point, local neighbour>.
         * In case of this matching method the metric describing neighour of the given point is 
         * distance to this neighbour
         */
        [](const pcl::PointXY &point, const pcl::PointXY &neighbour) { return utils::distance(point, neighbour); },

        /**
         * Metric used to compare pair <local neighbour, global neighbour> used to match neighbours
         * from both maps
         */
        [](const float &local, const float &global) { return std::abs(local - global); },

        /**
         * Maximal value of the mean metric calculated on corresponding neighbours that allows
         * matching local point with the global point
         */
        std_config.mean_neighbours_distance_threshold_m

    );
}


std::vector<GlobalLocalizationImpl::CorrsepondingPoints> GlobalLocalizationImpl::match_markers_with_neighbours_translation() const {
    return match_markers_with_neighbours_metric(

        /**
         * Intermediate callback calculating feature of pair <local point, local neighbour>.
         * In case of this matching method the metric describing neighour of the given point is 
         * translation to this neighbour
         */
        [](const pcl::PointXY &point, const pcl::PointXY &neighbour) { return utils::translation(point, neighbour); },

        /**
         * Metric used to compare pair <local neighbour, global neighbour> used to match neighbours
         * from both maps
         */
        [](const pcl::PointXY &local, const pcl::PointXY &global) { return utils::distance2(local, global); },

        /**
         * Maximal value of the mean metric calculated on corresponding neighbours that allows
         * matching local point with the global point. Note that the value is squared as 
         * a comparison metric used @ref distance2(...) function. This is only for sake of performance
         * to avoid calculating square roots
         */
        std::pow(std_config.mean_neighbours_translation_threshold_m, 2)

    );
}


template<typename NeighbourFeature, typename Metric>
std::vector<GlobalLocalizationImpl::CorrsepondingPoints> GlobalLocalizationImpl::match_markers_with_neighbours_metric(
    NeighbourFeature neighbour_feature,
    Metric metric,
    float mean_metric_limit
) const {

    std::vector<CorrsepondingPoints> corresponding_points;

    /**
     * Compute vector of neighbours' features for all local-map points (yes, this function is named
     * 'compute_all_metrics', it's not mistake)
     */
    auto local_neighbours_features = algorithm::compute_all_metrics(local_map_markers->points, std::forward<Metric>(metric));
    // Compute vector of neighbours' features for all global-map points
    auto global_neighbours_features = algorithm::compute_all_metrics(global_map_markers->points, std::forward<Metric>(metric));

    // Match points
    algorithm::match_overlapping(
        
        // Source/target markers
        local_map_markers->points,

        /**
         * Best-matching algorithm. Find the best global-map match for the given local-map point
         * based on features of matched points' neighbours
         */
        [&local_neighbours_features, &global_neighbours_features, &metric](
            std::size_t local_idx, const pcl::PointXY &local_point
        ) -> algorithm::CorrespondingElement<float> { 
            
            // Initialize current match with highe possible match cost
            algorithm::CorrespondingElement<float> current_match {
                .idx    = 0,
                .metric = std::numeric_limits<float>::max()
            };

            // Iterate over global-map points to find the one with the lowest match cost
            for(unsigned j = 0; j < global_neighbours_features.size(); ++j) {
            
                // Reset match cost to 0 for the new global-map point checked
                float match_cost = 0;

                /**
                 * Match each neighbour of the given local-map point to the global-map neighbour.
                 * At each match add metric calculated on corresponding features to the overall match
                 * cost.
                 */
                algorithm::match_overlapping_unordered(

                    // Source/target markers
                    local_neighbours_features[i],
                    global_neighbours_features[j],
                    // Metric
                    std::forward<Metric>(metric),

                    // Accumulator summing computed metrics into the match cost
                    [&match_cost](const algorithm::Correspondance<float> &match) { 
                        match_cost += match.metric;
                    }
                    
                );

                // If the matching cost for the global-map marker is lesser than the current-match cost, replace it
                if(current_match.metric > match_cost) {
                    current_match.idx    = j;
                    current_match.metric = match_cost;
                }

            }

        },

        // Accumulator
        [&corresponding_points, this](const algorithm::Correspondance<float> &match) { 
            corresponding_points.emplace_back( CorrsepondingPoints {
                .local  = local_map_markers->points[match.source_idx],
                .global = global_map_markers->points[match.target_idx]
            });
        },

        /**
         * Meteric limit for matched points (multiplied by number of neighbours as the
         * sum and not mean of matched-neighbours metrics is calculated)
         */
        mean_metric_limit * local_map_markers->size()
        
    );

    return corresponding_points;
}

/* ================================================= Private methods (registration) =============================================== */

tf2::Transform GlobalLocalizationImpl::estimate_robot_pose(const std::vector<CorrsepondingPoints> &corresponding_points) const {

    /**
     * @note Math rulling the following method has been explained at [1]. In general the method
     *    solves LSF (Least Square Fit) estimation of the 2D affine transformation matrix from
     *    'robot' to 'map' frame of reference based on the (expectedly) matching points in the
     *    'local' and 'global' map. This is done by standard computation based on computation of
     *    the Moore-Penrose (pseudo-inverse). 
     * @note Assume there is an affine transformation transforming points from 'robot' FoR
     *    (Frame of Reference) to the 'map' frame of reference described by hte matrix
     * 
     *    \f[
     *        R = \begin{bmatrix}
     *               cos(θ) & -sin(θ) & x \\
     *               sin(θ) &  cos(θ) & y \\
     *                 0    &    0    & 1
     *            \end{bmatrix}
     *    \f]
     * 
     *     transformation from PoI observed by the robot in it's local coordinate system and
     *     corresponding points in the 'global map' may be described as
     * 
     *     \f[
     *         \begin{bmatrix}
     *            g[0].x \\
     *            g[0].y \\
     *            g[1].x \\
     *            g[1].y \\
     *            \vdots \\
     *            g[N].x \\
     *            g[N].y
     *         \end{bmatrix} = 
     *         \begin{bmatrix}
     *            l[0].x & l[0].y &    1   &    0   &    0   & 0 \\
     *               0   &    0   &    0   & l[0].x & l[0].y & 1 \\
     *            l[1].x & l[1].y &    1   &    0   &    0   & 0 \\
     *               0   &    0   &    0   & l[1].x & l[1].y & 1 \\
     *                   &        & \vdots &        &        &   \\
     *            l[N].x & l[N].y &    1   &    0   &    0   & 0 \\
     *               0   &    0   &    0   & l[N].x & l[N].y & 1
     *         \end{bmatrix} \times
     *         \begin{bmatrix}
     *             cos(θ) \\
     *            -sin(θ) \\
     *               x    \\
     *             sin(θ) \\
     *             cos(θ) \\
     *               y   
     *         \end{bmatrix}
     *     \f]
     *
     *     wherew 'g[n]' is vector of points in the 'global map' and 'l[n]' is vector of corresponding
     *     points in the 'local map' After removing redundant multiplications one can transform the 
     *     equation in the brief form
     *     
     *     \f[
     *         \begin{array}{c}
     *             P \\
     *             \begin{bmatrix}
     *                g[0].x \\
     *                g[0].y \\
     *                g[1].x \\
     *                g[1].y \\
     *                \vdots \\
     *                g[N].x \\
     *                g[N].y
     *             \end{bmatrix} = 
     *         \end{array}
     *         \begin{array}{c}
     *             M \\
     *             \begin{bmatrix}
     *                l[0].x &  l[0].y & 1 & 0 \\
     *                l[0].y & -l[0].x & 0 & 1 \\
     *                l[1].x &  l[1].y & 1 & 0 \\
     *                l[1].y & -l[1].x & 0 & 1 \\
     *                       &  \vdots &   &   \\
     *                l[N].x &  l[N].y & 1 & 0 \\
     *                l[N].y & -l[N].x & 0 & 1
     *             \end{bmatrix} \times
     *         \end{array}
     *         \begin{array}{c}
     *             T \\
     *             \begin{bmatrix}
     *                 cos(θ) \\
     *                -sin(θ) \\
     *                   x    \\
     *                   y
     *             \end{bmatrix}
     *         \end{array}
     *     \f]         
     *    
     *     The problem can be solved by the standard equation utilizing Moore-Penrose pseudo-inversion of 
     *     the M matrix
     *     
     *     \f[
     *         T = M⁺ * P
     *     \f]
     *  
     *     where 
     *    
     *   \f$T\f$ - [4 x 1] vector describing unknown affine transformation; elements of this vector are
     *  \f$M⁺\f$ - Moore–Penrose inverse (pseudoinverse) of some [2*N x 4] M matrix whose components
     *             cotain coordinates of 'global map' markers
     *   \f$P\f$ - 'projection' vector of size [2*N x 1] containing coordinates of 'local map' markers
     *   \f$N\f$ - number of points to be matched
     * 
     * 
     * @see [1] https://stackoverflow.com/questions/11687281/transformation-between-two-set-of-points
     * 
     * @note In comparison to the original implementation by Wojtek Dudek role of @p markers_matched
     *   and @p map_markers_matched vectors in the solved equation has been reversed. I.e. in the oroginal
     *   code the @a M matrix contained coordinates of points in the 'global map' and @a P vector contained
     *   element contained coordinates of points in the 'local map'. In such a setup the affine transformation
     *   described by the T vector transforms points from 'global map' to the 'local map's frame of refernce
     *   (i.e. 'robot' frame of reference). In the localization algorithm one actually needs the oppositione
     *   transformation (i.e. from 'robot' to 'map' frame) at the same time describes pose of the robot in the
     *   'map' frame of reference 
     */

    Eigen::MatrixX4d M { corresponding_points.size() * 2, 4 };
    Eigen::VectorXd  P { corresponding_points.size() * 2    };

    // Fill both M matrix and P vector according to the described scheme
    for(unsigned i = 0; i < corresponding_points.size(); ++i) {

        // Fill corresponding rows of M matrix
        M.row(2 * i    ) << corresponding_points[i].local.x,  corresponding_points[i].local.y, 1.0, 0.0;
        M.row(2 * i + 1) << corresponding_points[i].local.y, -corresponding_points[i].local.x, 0.0, 1.0;
        // Fill corresponding rows of P vector
        P(2 * i    ) = corresponding_points[i].global.x;
        P(2 * i + 1) = corresponding_points[i].global.y;

    }

    // Solve the equation
    Eigen::Vector4d T = M.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(P);

    tf2::Transform robot_to_map_transform;

    // Parse translation of the result to the TF tranformation
    robot_to_map_transform.setOrigin(tf2::Vector3{ T(2), T(3), 0.0 });
    // Parse translation of the result to the TF tranformation
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, std::atan2( -T(1), T(0) ));
    robot_to_map_transform.setRotation(q);

    return robot_to_map_transform;
}

/* ================================================================================================================================ */

} // End namespace velmwheel 
