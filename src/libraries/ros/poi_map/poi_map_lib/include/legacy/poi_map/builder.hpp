/* ============================================================================================================================ *//**
 * @file       builder.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 31st March 2022 3:27:14 pm
 * @modified   Wednesday, 25th May 2022 5:23:12 pm
 * @project    engineering-thesis
 * @brief      Declaration of the poi_map::Builder class implementing routines focusing on building an PoI map
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __POI_MAP_BUILDER_H__
#define __POI_MAP_BUILDER_H__

/* =========================================================== Includes =========================================================== */

// System includes
#include <string>
#include <optional>
// ROS includes
#include "std_msgs/msg/color_rgba.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"
// TF includes
#include "tf2/LinearMath/Transform.h"
// Private includes
#include "poi_map_msgs/msg/point2_d.hpp"
#include "poi_map_msgs/msg/marker.hpp"

/* ================================================================================================================================ */

namespace poi_map {

/* ========================================================= Dependencies ========================================================= */

/**
 * @brief Score-update strategy of the Builder class. Calculates new score
 *    of the marker by incrementing it's old score by @c 1 up to the limit
 */
struct ScoreUpdateStrategy {

    /**
     * @brief Construct a new ScoreUpdateStrategy object
     * 
     * @param limit 
     *    maximum value
     */
    inline ScoreUpdateStrategy(double limit = 100'000);

    /**
     * @brief Calculates new score of the marker by incrementing it's old score by @c 1
     * 
     * @param[in] local_marker
     *    local marker
     * @param[in] incoming_maarker
     *    incoming marker
     * @returns 
     *    score of the @p local_marker incremented by @c 1
     */
    inline double update_score(
        const poi_map_msgs::msg::Marker& local_marker,
        const poi_map_msgs::msg::Marker& incoming_maarker
    ) const;

private:

    /// Limit of the marker's score
    const double limit;

};


/**
 * @brief Position-update strategy of the Builder class. Calculates new positon
 *    of the marker by moving it's old position half a way toward the incoming marker
 */
struct PositionUpdateStrategy {

    /**
     * @brief Construct a new PositionUpdateStrategy object
     */
    PositionUpdateStrategy() = default;

    /**
     * @brief Calculates new positon of the marker by moving it's old position half a way
     *    toward the incoming marker
     * 
     * @param[in] local_marker
     *    local marker
     * @param[in] incoming_maarker
     *    incoming marker
     * @returns 
     *    new position of the @p local_marker
     */
    inline poi_map_msgs::msg::Point2D update_position(
        const poi_map_msgs::msg::Marker& local_marker,
        const poi_map_msgs::msg::Marker& incoming_maarker
    ) const;
};


/**
 * @brief Representation-height strategy of the Builder class. Calculates height of
 *     the RVIZ point representing the marker as a current score of the point
 */
struct RepresentationHeightStrategy {
    
    /**
     * @brief Construct a new RepresentationHeightStrategy object
     */
    RepresentationHeightStrategy() = default;

    /**
     * @brief Calculates height of the RVIZ point representing the marker as a current 
     *    score of the point
     * 
     * @param[in] local_marker
     *    local marker
     * @returns 
     *    height of the @p local_marker's representation in RVIZ
     */
    inline double height_of(const poi_map_msgs::msg::Marker &local_marker) const;
    
};

/* ============================================================= Class ============================================================ */

template<
    typename PointsMatchingStrategy,
    typename ScoreUpdateStrategy,
    typename PositionUpdateStrategy,
    typename RepresentationHeightStrategy
> class BuilderConstructor;

/**
 * @brief Class implementing routines focusing on managing an PoI map at runtime
 * @details An PoI map is is represented as an arbitrary list of 3D points describing 2D
 *    position of the point as first two components and it's score as a third component. The map
 *    can be represented also (mainly for visualization purpose) as  a visualization_msgs::msg::Marker
 *    structure. In such a case the components of each marker is stored in the @a points member of the 
 *    structure.
 * 
 *    The Builder class stores a local list of PoI markers of a limited size. These
 *    markers represent current PoI map. Position of markers are given in some fixed
 *    frame of reference. 
 * 
 *    As an input, the Builder class takes arbitrary lists of PoI points. These represent 
 *    points of inerest (characteristic points) in the robot's environment and are choosen
 *    by the upstream node. They are given in the robot's frame of reference.
 * 
 *    Each incoming set of points is compared against points in the local map and tested for 
 *    matching. Two points are considered to match if some predicate given on two points is
 *    satisfied [1]. If the incoming point is matched with the one present in the local map
 *    it's 'score' is amplified (according to some update strategy). The local version 
 *    of the point is also moved some distance toward the incoming point [2].
 * 
 *    If the incoming points of intereset is not found in the local map, it is added with the
 *    @c 0 score
 * 
 *    Class provides helper transformation methods that aim to covert both the current PoI
 *    map and last incoming list of characteristic points into the visualization_msgs::msg::Marker
 *    structure for simple visualization with Rviz. Score of each point is represented as it's
 *    Z coordinate.
 * 
 * @note [1] The predicate is given as a strategy pattern. At the moment, the default predicate
 *    is limitation of the metric given on two points on a preconfigured threshold. The choosen
 *    metric is a rectilinear distance between two points. 
 * @todo Ask Wojtek Dudek why such a metric has been chosen
 * @note [2] The translation vector is given as a strategy pattern. At the moment the default
 *    strategy is to move the point half a way towards the target point
 * 
 * @tparam PointsMatchingStrategy
 *     type defining a method named @b match() with the following signature:
 *     bool(const poi_map_msgs::msg::Marker&, const poi_map_msgs::msg::Marker&).
 *     The method should return @p true if two markers are expected to represent the same 
 *     point in space
 * @tparam ScoreUpdateStrategy
 *     type defining a method named @b update_score() with the following signature:
 *     double(const poi_map_msgs::msg::Marker&, const poi_map_msgs::msg::Marker&)
 *     returning new value of the marker's score based on the locally saved (the first) and
 *     incoming (the second) marker
 * @tparam PositionUpdateStrategy
 *     type defining a method named @b update_position() with the following signature:
 *     poi_map_msgs::msg::Point2D(const poi_map_msgs::msg::Marker&,
 *     const poi_map_msgs::msg::Marker&) returning new position of the marker based on 
 *     the locally saved (the first) and incoming (the second) marker
 * @tparam RepresentationHeightStrategy
 *     type defining a method named @b height_of() with the following signature:
 *     double(const poi_map_msgs::msg::Marker&) returning height of the RVIZ point 
 *     representing the marker
 * 
 * @see https://wiki.ros.org/rviz/DisplayTypes/Marker   
 * @see https://github.com/dudekw/poi_map_builder
 */
template<
    typename PointsMatchingStrategy,
    typename ScoreUpdateStrategy,
    typename PositionUpdateStrategy,
    typename RepresentationHeightStrategy
> class Builder {

public: /* ------------------------------------------------- Static methods ------------------------------------------------------- */

    /**
     * @brief Creates the generator object
     */
    static BuilderConstructor<
        PointsMatchingStrategy,
        ScoreUpdateStrategy,
        PositionUpdateStrategy,
        RepresentationHeightStrategy
    > make_constructor();

public: /* ----------------------------------------------- Public ctors & dtors --------------------------------------------------- */

    /**
     * @brief Construct a new Builder object with given parameters
     * 
     * @param markers_limit 
     *    maximal number of markers in the local PoI map
     * @param fixed_frame 
     *    name of the fixed world TF frame that the local markers are to be stored in reference to
     * @param robot_frame 
     *    name of the robot's local TF frame that incoming markers are referenced to
     * @param map_markers_namespace 
     *    name of the RVIZ markers namespace associated with markers present int he current PoI map
     * @param incoming_markers_namespace 
     *    name of the RVIZ markers namespace associated with last incoming PoI markers
     * @param map_markers_color 
     *    array containing RGBA tuple representing colour of the RVIZ points representing locally-saved markers 
     * @param incoming_markers_color 
     *    array containing RGBA tuple representing colour of the RVIZ points representing incoming markers 
     * @param points_matching_strategy 
     *    point matching strategy for the Builder
     * @param score_update_strategy 
     *    score update strategy for the Builder
     * @param position_update_strategy 
     *    position update strategy for the Builder
     * @param representation_height_strategy 
     *    representation height strategy for the Builder
     */
    Builder(
        std::size_t markers_limit,
        const std::string &fixed_frame,
        const std::string &robot_frame,
        const std::string &map_markers_namespace,
        const std::string &incoming_markers_namespace,
        const std::array<float, 4> &map_markers_color,
        const std::array<float, 4> &incoming_markers_color,
        const PointsMatchingStrategy &points_matching_strategy,
        const ScoreUpdateStrategy &score_update_strategy,
        const PositionUpdateStrategy &position_update_strategy,
        const RepresentationHeightStrategy &representation_height_strategy
    );

    /**
     * @brief Default copy constructor
     */
    Builder(const Builder &other) = default;

    /**
     * @brief Default move constructor
     */
    Builder(Builder &&other) = default;

public: /* -------------------------------------------------- Public methods ------------------------------------------------------ */

    /**
     * @brief Updates the local PoI map based on the incoming set of @p markers 
     * 
     * @param markers 
     *    set of incoming PoI markers
     * @param fixed_to_robot_transform 
     *    transformation between fixed and robot's frame of reference
     */
    void update_map(
        const std::vector<poi_map_msgs::msg::Marker>& markers,
        const tf2::Transform &fixed_to_robot_transform
    );

    /**
     * @returns 
     *    refernce to the current PoI map
     */
    const std::vector<poi_map_msgs::msg::Marker> &get_map() const;

    /**
     * @brief Set the new map
     * 
     * @param map 
     *    map to be set
     */
    void set_map(const std::vector<poi_map_msgs::msg::Marker> &map);

    /**
     * @param score_threshold 
     *    minimal score of the marker to be visualized
     * @returns 
     *     RVIZ representation of the currently built PoI map
     */
    visualization_msgs::msg::Marker get_map_visualization(
        double score_threshold = std::numeric_limits<double>::max()
    ) const;

    /**
     * @param score_threshold 
     *    minimal score of the marker to be visualized
     * @returns 
     *     RVIZ representation of the last received set of PoI markers
     */
    visualization_msgs::msg::Marker get_current_markers_visualization(
        double score_threshold = std::numeric_limits<double>::max()
    ) const;

public: /* ------------------------------------------- Public visualization methods ----------------------------------------------- */

    /**
     * @brief Produces an RVIZ visualization of the given list of PoI markers
     * 
     * @param markers 
     *    list of markers to be transformed
     * @param rgba_color 
     *    colour of representation points
     * @param frame 
     *    reference frame of the given list of markers
     * @param ns 
     *    target namespace of the markers' representation
     * @param score_threshold 
     *    minimal score of the marker to be visualized
     * @returns 
     *    an RVIZ visualization of the given list of PoI markers
     */
    visualization_msgs::msg::Marker markers_to_visualization(
        const std::vector<poi_map_msgs::msg::Marker>& markers,
        const std::array<float, 4> &rgba_color, 
        const std::string &reference_frame,
        const std::string &ns,
        double score_threshold
    ) const;

public: /* ---------------------------------------------- Object's configuration -------------------------------------------------- */

    /**
     * @brief Auxiliary constants used to index @a *_color arrays
     */
    enum Color : unsigned {
        R = 0,
        G = 1,
        B = 2,
        A = 3
    };

    /// Name of the fixed world TF frame that the local markers are to be stored in reference to
    const std::string fixed_frame;
    /// Name of the robot's local TF frame that incoming markers are referenced to
    const std::string robot_frame;
    /// Name of the RVIZ markers namespace associated with markers present int he current PoI map
    const std::string map_markers_namespace;
    /// Name of the RVIZ markers namespace associated with last incoming PoI markers
    const std::string incoming_markers_namespace;
    
    /// Array containing RGBA tuple representing colour of the RVIZ points representing locally-saved markers 
    const std::array<float, 4> map_markers_color;
    /// Array containing RGBA tuple representing colour of the RVIZ points representing incoming markers 
    const std::array<float, 4> incoming_markers_color;
    
    /// Point matching strategy for the Builder
    const PointsMatchingStrategy points_matching_strategy;
    /// Score update strategy for the Builder
    const ScoreUpdateStrategy score_update_strategy;
    /// Position update strategy for the Builder
    const PositionUpdateStrategy position_update_strategy;
    /// Representation height strategy for the Builder
    const RepresentationHeightStrategy representation_height_strategy;

public: /* -------------------------------------------------- Object's state ------------------------------------------------------ */

    /// Set of incoming markers received on the last update
    std::vector<poi_map_msgs::msg::Marker> last_markers_received;
    /// Set of markers constituting the current map
    std::vector<poi_map_msgs::msg::Marker> map;

};

/* ======================================================= Auxiliary classes ====================================================== */

/**
 * @brief Auxiliary 'builder' class for the Builder
 */
template<
    typename PointsMatchingStrategy,
    typename ScoreUpdateStrategy,
    typename PositionUpdateStrategy,
    typename RepresentationHeightStrategy
> class BuilderConstructor {
    
public:

    /**
     * @brief Constructs a new BuilderConstructor object
     */
    BuilderConstructor() = default;

public:
    
    /// @brief Sets the maximal number of markers in the local PoI map
    inline BuilderConstructor &markers_limit(std::size_t limit);

    /// @brief Sets the target name of the fixed frame
    inline BuilderConstructor &fixed_frame(const std::string& frame);
    /// @brief Sets the target name of the robot's local frame
    inline BuilderConstructor &robot_frame(const std::string& frame);
    /// @brief Sets the target name of the RVIZ markers namespace associated with markers present int he current PoI map
    inline BuilderConstructor &map_markers_namespace(const std::string& ns);
    /// @brief Sets the target name of the RVIZ markers namespace associated with last incoming PoI markers
    inline BuilderConstructor &incoming_markers_namespace(const std::string& ns);
    
    /// @brief Sets the target colour of the RVIZ points representing locally-saved markers 
    inline BuilderConstructor &map_markers_color(const std::array<float, 4>& rgba_color);
    /// @brief Sets the target colour of the RVIZ points representing incoming markers 
    inline BuilderConstructor &incoming_markers_color(const std::array<float, 4>& rgba_color);

    /// @brief Sets the target points matching strategy
    inline BuilderConstructor &points_matching_strategy(const PointsMatchingStrategy& strategy);
    /// @brief Sets the target score update strategy
    inline BuilderConstructor &score_update_strategy(const ScoreUpdateStrategy& strategy);
    /// @brief Sets the target position update strategy
    inline BuilderConstructor &position_update_strategy(const PositionUpdateStrategy& strategy);
    /// @brief Sets the target representation height strategy
    inline BuilderConstructor &representation_height_strategy(const RepresentationHeightStrategy& strategy);
    
public:

    /**
     * @brief Builds the Builder class
     * 
     * @throws std::runtime_error
     *    if any of Builder's parameters has not been initialized
     */
    Builder<
        PointsMatchingStrategy,
        ScoreUpdateStrategy,
        PositionUpdateStrategy,
        RepresentationHeightStrategy
    > operator*() const;

private:

    /// Maximal number of markers in the local PoI map
    std::optional<std::size_t> markers_limit_;

    /// Name of the fixed world TF frame that the local markers are to be stored in reference to
    std::optional<std::string> fixed_frame_;
    /// Name of the robot's local TF frame that incoming markers are referenced to
    std::optional<std::string> robot_frame_;
    /// Name of the RVIZ markers namespace associated with markers present int he current PoI map
    std::optional<std::string> map_markers_namespace_;
    /// Name of the RVIZ markers namespace associated with last incoming PoI markers
    std::optional<std::string> incoming_markers_namespace_;

    /// Array containing RGBA tuple representing colour of the RVIZ points representing locally-saved markers 
    std::optional<std::array<float, 4>> map_markers_color_;
    /// Array containing RGBA tuple representing colour of the RVIZ points representing incoming markers 
    std::optional<std::array<float, 4>> incoming_markers_color_;

    /// Point matching strategy for the Builder
    std::optional<PointsMatchingStrategy> points_matching_strategy_;
    /// Score update strategy for the Builder
    std::optional<ScoreUpdateStrategy> score_update_strategy_;
    /// Position update strategy for the Builder
    std::optional<PositionUpdateStrategy> position_update_strategy_;
    /// Representation height strategy for the Builder
    std::optional<RepresentationHeightStrategy> representation_height_strategy_;

};

/* ================================================================================================================================ */

} // End namespace poi_map

/* ==================================================== Implementation includes =================================================== */

#include "poi_map/builder/builder.hpp"

/* ================================================================================================================================ */

#endif
