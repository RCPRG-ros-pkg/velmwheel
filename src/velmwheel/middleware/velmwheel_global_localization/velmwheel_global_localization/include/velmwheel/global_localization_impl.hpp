/* ============================================================================================================================ *//**
 * @file       global_localization_impl.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Friday, 1st April 2022 8:37:17 pm
 * @modified   Thursday, 26th May 2022 2:40:46 am
 * @project    engineering-thesis
 * @brief      Declaration fo the GlobalLocalizationImpl class implementing set of algorithms utilized by the 
 *             GlobalLocalization ROS node class
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __VELMWHEEL_GLOBAL_LOCALIZATION_IMPL_H__
#define __VELMWHEEL_GLOBAL_LOCALIZATION_IMPL_H__

/* =========================================================== Includes =========================================================== */

// System includes
#include <optional>
#include <vector>
// TF includes
#include "tf2/LinearMath/Transform.h"
// Interfaces includes
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "poi_map_msgs/msg/point2_d.hpp"
#include "poi_map_msgs/msg/marker.hpp"
#include "poi_map_msgs/msg/markers_stamped.hpp"
#include "poi_map_msgs/msg/markers_map.hpp"
// PCL includes
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

/* ========================================================== Namespaces ========================================================== */

namespace velmwheel {

/* ============================================================= Class ============================================================ */

/**
 * @brief Class implementing set of algorithms utilized by the GlobalLocalization ROS node
 *    class based on one of two technics:
 * 
 *      * @b STD - a custom localization algorithm based on aproximated matching of 'local' and
 *                 'global' markers in the robot's environment
 *      * @b ICP - localization method based on the ICP algorithm finding transformation between
 *                 'local' and 'global' markers in the robot's environment
 * 
 * @note Comments of the GlobalLocalizationImpl implementation use two terms refering to specific
 *    FoR (Frames of Reference) - 'map' and 'odom'. These are not exactly names of concrete TF
 *    frames but rather representations of two concepts - reference frame of the markers present
 *    in the PoI map and reference frame of the odometry data. There is also reference to the 'robot'
 *    frame. This one refers to the concept of the locally-fixed frame in the robot's frame of reference
 */
class GlobalLocalizationImpl {

public: /* --------------------------------------------------- Public types -------------------------------------------------------- */

    /**
     * @brief Configuration of the STD localization algorithm
     * @details STD localization method
     */
    struct ConfigSTD {
        
        /**
         * @brief Methods of matching 'local' markers with 'global' markers
         */
        enum class MatchingMethod {

            /**
             * @brief This is the default method of markers-matching in the STD localization mode.
             *    When used, the algorithm transforms 'local' markers in to the 'global' frame of reference
             *    ('map' frame) using transformation computed in the previous iteration
             * 
             * @note As this method depends on the result of the previous iteration of localization algorithm
             *    it must have another matching method set as fallback. If this rule is not respected the class'
             *    constructor will reset the fallback method to @c MatchingMethod::PreviousTransform
             */
            PreviousTransform,

            /**
             * @brief Markers-matching method pairing a 'local' marker (aka 'local reference point') with such 
             *   a 'global' marker (aka 'global reference point') that minimizes mean difference between
             *   
             *      - distance calculated between the local reference point and it's neighbour
             *      - distance calculated between the global reference point and it's supposingly
             *        corresponding neighbour choosen under assumption that local reference
             *        point and global reference point represent the same point in space
             * 
             *    A single neighbour of the global reference point can be matched with many neighbours of the
             *    local reference point.
             * 
             *    Two points are matched when the minimized difference is not higher than 
             *    @a mean_neighbours_distance_threshold_m .
             * 
             * @todo Implement 'non-overlaping nieghbours' version of the matching pattern
             */
            NeighboursDistance,

            /**
             * @brief Markers-matching method pairing a 'local' marker (aka 'local reference point') with such 
             *   a 'global' marker (aka 'global reference point') that minimizes mean Euklidean distance between
             *   
             *      - a neighbour of the local reference point and 
             *      - supposingly corresponding neighbour of the global reference point choosen
             *        under assumption that local reference point and global reference point 
             *        represent the same point in space
             * 
             *    A single neighbour of the global reference point can be matched with many neighbours of the local
             *    reference point.
             * 
             *    Two points are matched when the minimized difference is not higher than 
             *    @a mean_neighbours_translation_threshold_m .
             * 
             * @todo Implement 'non-overlaping nieghbours' version of the matching pattern
             */
            NeighboursTranslation

        };

        /// Default method of markers-matching
        MatchingMethod matching_method { MatchingMethod::PreviousTransform };
        /// Fallback method of markers-matching
        MatchingMethod fallback_matching_method { MatchingMethod::NeighboursDistance };

        /**
         * @brief Maximal distance between estimated position of the incoming marker in the 'map' frame of reference
         *    and position of the compared map marker for which node assumes that both points correspond to each 
         *    other in [m] used when the @c MatchingMethod::PreviousTransform markers-matching method is used
         */
        double previous_transform_match_threshold { 0.25 };

        /**
         * @brief When markers-matching method pairing a 'local' marker (aka 'local reference point') with such 
         *   a 'global' marker (aka 'global reference point') that minimizes mean difference between
         *   
         *      - Euklidean distance calculated between the local reference point and it's 
         *        neighbour
         *      - Euklidean distance calculated between the global reference point and it's 
         *        supposingly corresponding neighbour choosen under assumption that local 
         *        reference point and global reference point represent the same point in space
         * 
         *    this is the maximal value of the minimized difference that allows matching both markers
         */
        double mean_neighbours_distance_threshold_m { 0.15 };

        /**
         * @brief When markers-matching method pairing a 'local' marker (aka 'local reference point') with such 
         *   a 'global' marker (aka 'global reference point') that minimizes minimizes mean Euklidean distance between
         *   
         *      - a neighbour of the local reference point and 
         *      - supposingly corresponding neighbour of the global reference point choosen
         *        under assumption that local reference point and global reference point 
         *        represent the same point in space
         * 
         *    this is the maximal value of the minimized distance that allows matching both markers
         */
        double mean_neighbours_translation_threshold_m { 0.15 };

    };

    /**
     * @brief Configuration of the ICP-based localization algorithm
     */
    struct ConfigICP {

        /// Maximum correspondance distance for accepted match pairs
        double max_correspondance_distance { 100.0 };
        /// Epsilon (maximal accepted inter-iteration change) in the ICP transformation
        double transformation_epsilon { 1e-10 };
        /// Epsilon (maximal accepted inter-iteration change) in the fitness of point clouds
        double euclidean_fitness_epsilon { 0.001 };
        /// Maximum number of tierations of the ICP algorithm
        unsigned maximum_iterations { 100 };
        
    };  

public: /* --------------------------------------------------- Public ctors -------------------------------------------------------- */

    /**
     * @brief Construct a new GlobalLocalizationImpl subsystem with the given configuration
     * 
     * @param std_config
     *    configuration of the STD localization algorithm 
     * @param icp_config
     *    configuration  of the ICP-based localization algorithm
     */
    GlobalLocalizationImpl(
        const ConfigSTD &std_config,
        const ConfigICP &icp_config
    );

public: /* ---------------------------------------------------- Public API -------------------------------------------------------- */

    /**
     * @brief Updates odometry data about the robot
     * 
     * @param msg 
     *    incoming odometry data
     */
    void update_odometry(const nav_msgs::msg::Odometry &msg);

    /**
     * @brief Updates 'local map' of PoI markers 
     * 
     * @param msg 
     *    incoming markers map
     */
    void update_scan(const poi_map_msgs::msg::MarkersStamped &msg);

    /**
     * @brief Updates 'global map' of PoI markers 
     * 
     * @param msg 
     *    incoming markers map
     */
    void update_map(const poi_map_msgs::msg::MarkersMap &msg);
    
    /**
     * @brief Based on the currently cached odometry and PoI (points of interest) data position of
     *    the 'odom' FoR (Frame of Reference) - i.e. FoR of incoming odometry data - in the 'map' 
     *    FoR being a reference source for markers in the 'global map'
     * @details Based on the PoI-matching algorithm method tries to calculate estimated pose of the
     *    'robot' in the 'map' frame. On success it combines resulting transformation with the 
     *    'robot' -> 'odom' transformation obtained from odometry data to calculate estimated
     *    transformation from 'odom' frame to the 'map' frame
     * 
     * @returns 
     *    @retval current estimation of the transformation from the 'odom' frame to the 'map' frame
     *       (i.e. pose of the 'odom' frame in the 'map' frame of reference) on success
     *    @retval an empty std::optional on failure
     */
    std::optional<tf2::Transform> estimate_odom_pose_std();
    
    /**
     * @brief Based on the currently cached odometry and PoI (points of interest) data position of
     *    the 'odom' FoR (Frame of Reference) - i.e. FoR of incoming odometry data - in the 'map' 
     *    FoR being a reference source for markers in the 'global map'
     * @details Finds transformation from 'robot' to 'map' FoR using PCL (Point Cloud Library) 
     *    implementation of ICP algorithm ( @see [1] ). Uses it to find 'odom' to 'map' transformation
     *    using estimated transformation form 'robot' to 'odom' obtained with odometry
     * 
     * @returns 
     *    @retval current estimation of the transformation from the 'odom' frame to the 'map' frame
     *       (i.e. pose of the 'odom' frame in the 'map' frame of reference) on success
     *    @retval an empty std::optional on failure
     * 
     * @see [1] https://en.wikipedia.org/wiki/Iterative_closest_point
     */
    std::optional<tf2::Transform> estimate_odom_pose_icp();

private: /* --------------------------------------------- Private methods (common) ------------------------------------------------ */

    /**
     * @brief Using provided transformation from 'robot' to 'map' FoR finds transformation from 
     *    'odom' to 'map' using estimated transformation form 'robot' to 'odom' obtained with
     *    odometry
     * 
     * @param robot_to_map_transform
     *    transformation from 'robot' to 'map' FoR (if empty optional given, function immediatelly
     *    returns an empty optional)
     * 
     * @retval estimation 
     *    current estimation of the transformation from the 'odom' frame to the 'map' frame
     *    (i.e. pose of the 'odom' frame in the 'map' frame of reference) on success
     * @retval empty
     *    an empty std::optional if empty @p robot_to_map_transform given
     */
    std::optional<tf2::Transform> estimate_odom_pose(const std::optional<tf2::Transform> &robot_to_map_transform) const;
    

private: /* ------------------------------------------ Private types (STD algorithm) ---------------------------------------------- */

    /**
     * @brief Auxiliary pair-like structure holding positions of matching markers
     *    from the 'global' and 'local' PoI maps (both given in the 'map' frame of 
     *    reference) 
     */
    struct CorrsepondingPoints {

        /// Positiion of the marker from the 'local map'
        pcl::PointXY local;
        /// Positiion of the marker from the 'global map'
        pcl::PointXY global;
        
    };
    
private: /* ---------------------------------- Private methods - localization (STD algorithm) ------------------------------------- */

    /**
     * @brief Based on some standard PoI-matching algorithm tries to calculate estimated pose of the
     *    'robot' in the 'map' frame
     * 
     * @retval estimation
     *    current estimation of the transformation from the 'robot' frame to the 'map' frame
     *    (i.e. pose of the 'robot' frame in the 'map' frame of reference) on success
     * @retval empty
     *    an empty std::optional on failure
     */
    std::optional<tf2::Transform> localize_std();

private: /* -------------------------------- Private methods - markers-matching (STD algorithm) ----------------------------------- */

    /**
     * @brief Tries to match local and global markers using given @p match_type
     * 
     * @param match_type 
     *    markers-match method to be used
     * @returns 
     *    vector of corresponding points
     */
    std::vector<CorrsepondingPoints> match_markers(ConfigSTD::MatchingMethod match_type) const;

    /**
     * @brief Matches markers from the 'local map' with markers from the 'global map' based on the current
     *    estimation of the transformation from the 'robot' FoR to the 'map' FoR. Matches each point from 
     *    the 'local map' with it's closest neighbour from the 'global map' providing that estimated
     *    distance between two is not higher than @a local_match_threshold . Otherwise consider the 'local'
     *    marker unmatched and does not place in the result list.
     * 
     * @tparam allow_overlaping 
     *    configuration of whether single 'global' markers can be matched to many 'local' markers
     * @returns 
     *     pair of vectors of corresponding points
     */
    std::vector<CorrsepondingPoints> match_markers_with_previous_transform() const;

    /**
     * @brief pairing a 'local' marker (aka 'local reference point') with such 
     *   a 'global' marker (aka 'global reference point') that minimizes mean difference between
     *   
     *      - distance calculated between the local reference point and it's neighbour
     *      - distance calculated between the global reference point and it's supposingly
     *        corresponding neighbour choosen under assumption that local reference
     *        point and global reference point represent the same point in space
     * 
     *    A single neighbour of the global reference point can be matched with many neighbours of the
     *    local reference point.
     * 
     *    Two points are matched when the minimized difference is not higher than 
     *    @a mean_neighbours_distance_threshold_m .
     * 
     * @returns 
     *     pair of vectors of corresponding points
     */
    std::vector<CorrsepondingPoints> match_markers_with_neighbours_distance() const;

    /**
     * @brief Matches markers pairing a 'local' marker (aka 'local reference point') with such 
     *   a 'global' marker (aka 'global reference point') that minimizes mean Euklidean distance between
     *   
     *      - a neighbour of the local reference point and 
     *      - supposingly corresponding neighbour of the global reference point choosen
     *        under assumption that local reference point and global reference point 
     *        represent the same point in space
     * 
     *    A single neighbour of the global reference point can be matched with many neighbours of the local
     *    reference point.
     * 
     *    Two points are matched when the minimized difference is not higher than 
     *    @a mean_neighbours_translation_threshold_m .
     * 
     * @returns 
     *     pair of vectors of corresponding points
     */
    std::vector<CorrsepondingPoints> match_markers_with_neighbours_translation() const;

    /**
     * @brief Tries to match local-map markers with global-map markers based on set of features
     *    of paired points' neighbours.
     * @details For each local-map point to be matched function calculates vector of feature
     *    of other points in local map (aka 'neighbours'). Such a feature may be e.g. translation
     *    or distance between the given point and its' neighbour. Subsequently algorithm computes
     *    such features for each point in the global-map. To compute neighbour's feature the
     *    @p neighbour_feature callable is called with both - actual reference point and neighbout.
     * 
     *    For each global-map point, based on feature vectors algorithm tries to pair neighbours
     *    of the local- and global-map point in such a way that minimizes mean value of @p metric
     *    calculated on paired neighbours.
     *   
     *    Next, the overall cost of matching local- and global-map point is calculated as a mean
     *    metric calculated on paired neighbours. If such a metric is not greater than 
     *    @p mean_metric_limit the local point is matched.
     * 
     * @note In the original implementation by Wojtek Dudek there were four major differences to the
     *    described algorithm:
     *    
     *       1) Algorithm operated specifically on @p neighbour_feature being <b>distance</b> from the
     *          matched point to the neighbour and @p metric was <b>absolute difference</b> between
     *          features; this function provides a generalized approach to allow broader class of features
     * 
     *       2) Due to the 1) features vectors has been sorted (this is not possible in case of
     *          e.g. feature being a 2D translation between points); this has been exploited ( or at 
     *         least was intended to be, @see 3) ) in such a way that when algorithm has been iterating
     *         through vector of global-neighbours features (to find match for local-neighbour)
     *         it was checking only difference between current and previous @p metric and choosen
     *         to match first global-neighour for which this difference was positive (i.e. current
     *         global-neighbour was first 'worse' match for the local-neighbour than the last match)
     * 
     *       4) single global-point-neighbour could not be matched with many local-point-neighbours
     *          ( at least that was probable intention; @bug due to the (probable) bug in code most of 
     *          local-point neighbours has been very often simultaneously matched to the neighbour
     *          associated with the last feature in the global-neighbours-feaures list )
     * 
     *       4) If the local-map point could not be matched with the global-map point it has been removed
     *          from the matching set and the matching process for the whole map (including recalculation
     *          of vectors of features) has been restarted
     * 
     *    These properties of the original algorithm seem to be either result of code bugs or an arbitrary
     *    choice of the maching method that 'it will be working in some way, in the end'. This is justtified
     *    in the perspective of the fact that this matching algorithm has been used as a fall-back for the
     *    previous-transform-based ( @ref match_markers_with_previous_transform ) algorithm and was not used
     *    most of the time.
     * 
     * @todo Verify whether these changes in fact were not crucial and do not broke the global localization
     *    subsystem
     * 
     * @tparam NeighbourFeature
     *    type of the callable used to calculate feature of the point's neighbour
     * @tparam Metric
     *    type of the callable used to calculate 'distance' between two features
     * @param neighbour_feature 
     *    callable used to calculate feature of the point's neighbour; it should take two @ref pcl::PointXY
     *    obects as arguments and return some feature value; the first argument is the reference point and
     *    the second is the neighbour
     * @param neighbour_feature 
     *    callable used to calculate calculate 'distance' between two features; it should take two objects
     *    of type returned by the @p neighbour_feature callable and return floating-point value
     * @param mean_metric_limit 
     *    mean value of the @p metric calclated on paired neighbours of local- and global-map point that
     *    allows matching these points
     */
    template<typename NeighbourFeature, typename Metric>
    std::vector<CorrsepondingPoints> match_markers_with_neighbours_metric(
        NeighbourFeature neighbour_feature,
        Metric metric,
        float mean_metric_limit
    ) const;

private: /* ---------------------------------- Private methods - registration (STD algorithm) ------------------------------------- */

    /**
     * @brief Based on the PoI-matching algorithm method tries to calculate estimated pose of the
     *    'robot' in the 'map' frame
     * 
     * @param markers_matched
     *    list of PoIs from the current 'local map' that has been matched to some PoIs in the 'global map'
     * @param map_markers_matched 
     *    list of PoIs from the current 'global map' matching subsequent points given in the @p markers_matched list
     * 
     * @retval estimation 
     *    current estimation of the transformation from the 'robot' frame to the 'map' frame
     *    (i.e. pose of the 'robot' frame in the 'map' frame of reference) on success
     * @retval empty 
     *    an empty std::optional on failure
     * 
     * @note @p markers_matched list contains at most one copy of the point from the 'local map' while the
     *    @p map_markers_matched may contain more than on copy of elements from the 'global map'. This indicates
     *    that one point from the 'local map' has been matched to many points in the 'global map'
     * 
     * @note Both vectors must be of the same size
     */
    tf2::Transform estimate_robot_pose(const std::vector<CorrsepondingPoints> &corresponding_points) const;

private: /* ------------------------------------------ Private methods (ICP algorithm) -------------------------------------------- */

    /**
     * @brief Finds transformation from 'robot' to 'map' FoR using PCL (Point Cloud Library) 
     *    implementation of ICP algorithm ( @see [1] )
     * 
     * @retval estimation 
     *    current estimation of the transformation from the 'robot' frame to the 'map' frame
     *    (i.e. pose of the 'robot' frame in the 'map' frame of reference) on success
     * @retval empty
     *    an empty std::optional on failure
     * 
     * @see [1] https://en.wikipedia.org/wiki/Iterative_closest_point
     */
    std::optional<tf2::Transform> localize_icp();
    
private: /* ---------------------------------------------- Private configuration -------------------------------------------------- */


    /// Configuration of the STD localization algorithm
    const ConfigSTD std_config;
    /// Configuration of the ICP-based localization algorithm
    const ConfigICP icp_config;

private: /* ------------------------------------------------- Object's state ------------------------------------------------------ */

    /// Transformation from 'robot' to 'odom' (i.e. pose of the 'robot' in the 'odom') frame based on the last odometry data
    tf2::Transform robot_to_odom_transform { tf2::Transform::getIdentity() };
    /// Transformation from the 'odom' to 'robot' (i.e. pose of the 'odom' in the 'robot') frame based on the first to last odometry data
    tf2::Transform odom_to_robot_transform_previous { tf2::Transform::getIdentity() };

    /// List of intensity markers present in the global map
    pcl::PointCloud<pcl::PointXY>::Ptr global_map_markers { new pcl::PointCloud<pcl::PointXY> };
    /// List of intensity markers present in the local (temporary) map
    pcl::PointCloud<pcl::PointXY>::Ptr local_map_markers { new pcl::PointCloud<pcl::PointXY> };

    /**
     * @brief Last estimated transformation from the 'robot' frame to the 'map' frame obtained by solving PoI matching problem.
     *    This transformation is used to estimate pose of the 'local map' markers in the 'map' FoR so that they can be matched
     *    with markers in the 'global map'. In the first iteration of the localization algorithm this transformation is not
     *    available and so another method needs to be used to match points from both maps. For this reason the transformation
     *    is 'optional' to indicate whether the algorithm has been initialized (i.e. at least one 'robot' -> 'map' transformation
     *    has been estimated)
     */
    std::optional<tf2::Transform> last_robot_to_map_transform;

};

/* ================================================================================================================================ */

} // End namespace velmwheel

#endif
