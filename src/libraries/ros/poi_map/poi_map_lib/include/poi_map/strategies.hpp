/* ============================================================================================================================ *//**
 * @file       strategies.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 31st March 2022 3:27:14 pm
 * @modified   Wednesday, 25th May 2022 5:38:10 pm
 * @project    engineering-thesis
 * @brief      Declaration of common strategies types used across the 'poi_map_lib' package
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __POI_MAP_STRATEGIES_H__
#define __POI_MAP_STRATEGIES_H__

/* =========================================================== Includes =========================================================== */

// System includes
#include <optional>
// Private includes
#include "poi_map_msgs/msg/point2_d.hpp"
#include "poi_map_msgs/msg/marker.hpp"

/* ================================================================================================================================ */

namespace poi_map {

/* ========================================================= Base strategy ======================================================== */

/**
 * @brief Abstract strategy implementing markers-discrimnation algorithm. It's aim
 *    is to tell whether the incoming marker should be taken into account in the further
 *    processing of some wider algorithm
 */
struct BaseStrategy {

    /**
     * @brief Destroy the Base Strategy object
     */
    virtual ~BaseStrategy() = default;

    /**
     * @brief Reset method that can be called on the strategy object to inform
     *    underlying implementation that a new portion of incoming PoI
     *    markers is being processed()
     */
    virtual void reset() { };

};

/* ========================================================== Strategies ========================================================== */

/**
 * @brief Abstract strategy implementing markers-transformation algorithm. It's aim
 *    is to perform an (optional) initial transformation of the point before being
 *    processed
 */
struct TransformationStrategy : BaseStrategy {

    /**
     * @brief Destroy the Transformation Strategy object
     */
    virtual ~TransformationStrategy() = default;

    /**
     * @brief Transforms @p incoming marker based on properties of the wole batch
     * 
     * @param[in] incoming_marker
     *    incoming marker being processed
     * @param[in] all_incoming_markers
     *    all incoming markers in current batch
     * 
     * @retval marker
     *    transformed version of the marker on success
     * @retval empty 
     *    empty optional if the marker should not be processed
     */
    virtual std::optional<poi_map_msgs::msg::Marker> transform(
        const poi_map_msgs::msg::Marker& incoming_marker,
        [[maybe_unused]] const std::vector<poi_map_msgs::msg::Marker>& all_incoming_markers
    ) { return incoming_marker; };

};

/**
 * @brief Abstract strategy implementing markers-matching algorithm 
 */
struct MatchingStrategy : BaseStrategy {

    /**
     * @brief Destroy the Markers Matching Strategy object
     */
    virtual ~MatchingStrategy() = default;

    /**
     * @param[in] saved_marker
     *    local marker
     * @param[in] incoming_maarker
     *    incoming marker
     * 
     * @retval true 
     *    if given points match each other
     * @retval false 
     *    otherwise
     */
    virtual bool match(
        const poi_map_msgs::msg::Marker& saved_marker,
        const poi_map_msgs::msg::Marker& incoming_maarker
    ) = 0;

};


/**
 * @brief Abstract strategy implementing post-matching markers-score-update algorithm 
 */
struct ScoreUpdateStrategy : BaseStrategy {

    /**
     * @brief Destroy the Score Update Strategy object
     * 
     */
    virtual ~ScoreUpdateStrategy() = default;

    /**
     * @brief Calculates new score of the @p saved_marker marker assuming it has been matched with
     *    the @p incoming_marker
     * 
     * @param[in] saved_marker
     *    local marker
     * @param[in] incoming_maarker
     *    incoming marker
     * @returns 
     *    new score of the @p local_marker
     */
    virtual double update(
        const poi_map_msgs::msg::Marker& saved_marker,
        const poi_map_msgs::msg::Marker& incoming_maarker
    ) = 0;

};


/**
 * @brief Abstract strategy implementing post-matching markers-position-update algorithm
 */
struct PositionUpdateStrategy : BaseStrategy {

    /**
     * @brief Destructrs a new Position Update Strategy object
     */
    virtual ~PositionUpdateStrategy() = default;

    /**
     * @brief Calculates new positon of the @p saved_marker marker assuming it has been matched with
     *    the @p incoming_marker
     * 
     * @param[in] saved_marker
     *    local marker
     * @param[in] incoming_maarker
     *    incoming marker
     * @returns 
     *    new position of the @p saved_marker
     */
    virtual poi_map_msgs::msg::Point2D update(
        const poi_map_msgs::msg::Marker& saved_marker,
        const poi_map_msgs::msg::Marker& incoming_maarker
    ) = 0;
    
};


/**
 * @brief Abstract strategy implementing discarding algorithm for list of markers. Such a strategy
 *    can be used e.g. by the PoI-map-builder entity to remove out-of-date markers from the
 *    current map
 */
struct DiscardingStrategy : BaseStrategy {

    /**
     * @brief Destructrs a new Discarding Strategy object
     */
    virtual ~DiscardingStrategy() = default;

    /**
     * @brief Discards out-of-date markers from the @p markers list
     * 
     * @param[in] markers
     *    list of markers to be processed
     * @returns 
     *    updated @p markers list
     */
    virtual void discard(
        [[maybe_unused]] std::vector<poi_map_msgs::msg::Marker>& markers
    ) { return; };
    
};

/* ================================================================================================================================ */

} // End namespace poi_map

/* ================================================================================================================================ */

#endif
