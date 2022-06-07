/* ============================================================================================================================ *//**
 * @file       map_builder.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 31st March 2022 3:27:14 pm
 * @modified   Wednesday, 25th May 2022 5:38:20 pm
 * @project    engineering-thesis
 * @brief      Declaration of the poi_map::MapBuilder class implementing routines focusing on managing an PoI map at runtime
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __POI_MAP_MAP_BUILDER_H__
#define __POI_MAP_MAP_BUILDER_H__

/* =========================================================== Includes =========================================================== */

// System includes
#include <string>
#include <optional>
#include <memory>
// Private includes
#include "poi_map/strategies.hpp"

/* ================================================================================================================================ */

namespace poi_map {

/* ============================================================= Class ============================================================ */

class MapBuilderConstructor;

/**
 * @class MapBuilder
 * @brief Class implementing routines building an PoI map basing on incoming 
 *    point-of-interests (POI)
 * @details An PoI map is is represented as an arbitrary list of 3D points describing 2D
 *    position of the point as first two components and it's score as a third component. 
 * 
 *    The Builder class stores a local list of PoI markers of a limited size. These
 *    markers represent current PoI map. Position of markers are given in some fixed
 *    frame of reference. 
 * 
 *    As an input, the Builder class takes arbitrary lists of PoI points. These represent 
 *    points of inerest (characteristic points) in the robot's environment and are choosen
 *    by the upstream agent.
 * 
 *    Each incoming point is initially transformed due to some preconfigured strategy and compared
 *    against points in the local map and tested for matching. Two points are considered to match
 *    if some (injected at construct-time) predicate given on two points is satisfied. If the 
 *    incoming point is matched with the one present in the local map it's 'score' is amplified 
 *    (according to some update strategy). The local version of the point is also moved some 
 *    distance toward the incoming point.
 * 
 *    If the incoming points of intereset is not found in the local map, it is added with the
 *    @c 0 score
 * 
 * @note MapBuilder class implementation is based on the Abstract Algorithm pattern
 * @performance Currently strategies implementing steps of map-building algorithm are injected
 *    into the builder object vis @ref std::unique_ptr . This imposes usage of the dynamic
 *    memory which can hit performance if the map builder is used on the critical path of the
 *    control loop. In the future this problem can be addressed by switching from dynamic
 *    dependencies into static dependencies (i.e. making MapBuilder class template) as the
 *    builder in egneral does not require runtime dependencies switching.
 */
class MapBuilder {

public: /* ------------------------------------------------- Static methods ------------------------------------------------------- */

    /**
     * @brief Creates the 'builder-like' object for constructing the MapBuilder object
     */
    static MapBuilderConstructor make_constructor();

public: /* ----------------------------------------------- Public ctors & dtors --------------------------------------------------- */

    /**
     * @brief Construct a new MapBuilder object with given parameters
     * 
     * @param markers_limit 
     *    maximal number of markers in the local PoI map
     * @param transformation_strategy
     *     strategy used to transform incoming markers before matching it with the current map
     * @param matching_strategy
     *     markers-matching strategy deciding whether two markers (locally-saved in map and incoming one)
     *     are expected to represent the same point in space
     * @param score_update_strategy
     *     strategy object updating value of the in-map marker's score based on it's current properties
     *     and properties of the matched incoming marker
     * @param position_update_strategy
     *     strategy object updating value of the in-map marker's position based on it's current properties
     *     and properties of the matched incoming marker
     * @param discarding_strategy
     *     strategy object removing out-of-date markers from the map
     * 
     * @throws std::runtime_error 
     *     when any of given strategies is @p nullptr
     */
    MapBuilder(
        std::size_t markers_limit,
        std::unique_ptr<TransformationStrategy> transformation_strategy,
        std::unique_ptr<MatchingStrategy> matching_strategy,
        std::unique_ptr<ScoreUpdateStrategy> score_update_strategy,
        std::unique_ptr<PositionUpdateStrategy> position_update_strategy,
        std::unique_ptr<DiscardingStrategy> discarding_strategy
    );

    /**
     * @brief Default copy constructor
     */
    MapBuilder(const MapBuilder &other) = default;

    /**
     * @brief Default move constructor
     */
    MapBuilder(MapBuilder &&other) = default;

public: /* -------------------------------------------------- Public methods ------------------------------------------------------ */

    /**
     * @brief Updates the local PoI map based on the incoming set of @p markers 
     * 
     * @param markers 
     *    set of incoming PoI markers
     * 
     * @note Incoming markers are assumed to be given in the same frame of reference as
     *    the map's markers
     */
    void update(const std::vector<poi_map_msgs::msg::Marker>& markers);

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

private: /* --------------------------------------------- Object's configuration -------------------------------------------------- */
    
    /// Markers transformation strategy for the Builder
    std::unique_ptr<TransformationStrategy> transformation_strategy;
    /// Point matching strategy for the Builder
    std::unique_ptr<MatchingStrategy> matching_strategy;
    /// Score update strategy for the Builder
    std::unique_ptr<ScoreUpdateStrategy> score_update_strategy;
    /// Position update strategy for the Builder
    std::unique_ptr<PositionUpdateStrategy> position_update_strategy;
    /// Representation height strategy for the Builder
    std::unique_ptr<DiscardingStrategy> discarding_strategy;

private: /* ------------------------------------------------- Object's state ------------------------------------------------------ */

    /// Set of markers constituting the current map
    std::vector<poi_map_msgs::msg::Marker> map;

};

/* ======================================================= Auxiliary classes ====================================================== */

/**
 * @brief Auxiliary 'builder' class for the Builder
 */
class MapBuilderConstructor {
    
public:

    /**
     * @brief Constructs a new MapBuilderConstructor object
     */
    MapBuilderConstructor() = default;

public:
    
    /// @brief Sets the maximal number of markers in the local PoI map
    MapBuilderConstructor &markers_limit(std::size_t limit);

    /// @brief Sets the target markers-transformation strategy (optional)
    MapBuilderConstructor & transformation_strategy(std::unique_ptr<TransformationStrategy> strategy);
    /// @brief Sets the target markers-matching strategy
    MapBuilderConstructor &matching_strategy(std::unique_ptr<MatchingStrategy> strategy);
    /// @brief Sets the target score-update strategy
    MapBuilderConstructor &score_update_strategy(std::unique_ptr<ScoreUpdateStrategy> strategy);
    /// @brief Sets the target position-update strategy
    MapBuilderConstructor &position_update_strategy(std::unique_ptr<PositionUpdateStrategy> strategy);
    /// @brief Sets the target markers-discarding strategy (optional)
    MapBuilderConstructor &discarding_strategy(std::unique_ptr<DiscardingStrategy> strategy);
    
public:

    /**
     * @brief Builds the MapBuilder class
     * 
     * @throws std::runtime_error
     *    if any of MapBuilder's parameters has not been initialized
     */
    MapBuilder operator*();

private:

    /// Maximal number of markers in the local PoI map
    std::optional<std::size_t> markers_limit_;

    /// Target markers-discrimnation strategy
    std::unique_ptr<TransformationStrategy> transformation_strategy_ { std::make_unique<TransformationStrategy>() };
    /// Target markers-matching strategy for the MapBuilder class
    std::unique_ptr<MatchingStrategy> matching_strategy_;
    /// Target score-update strategy for the MapBuilder class
    std::unique_ptr<ScoreUpdateStrategy> score_update_strategy_;
    /// Target position-update strategy for the MapBuilder class
    std::unique_ptr<PositionUpdateStrategy> position_update_strategy_;
    /// Target markers-discarding strategy for the MapBuilder class
    std::unique_ptr<DiscardingStrategy> discarding_strategy_ { std::make_unique<DiscardingStrategy>() };

};

/* ================================================================================================================================ */

} // End namespace poi_map

/* ================================================================================================================================ */

#endif
