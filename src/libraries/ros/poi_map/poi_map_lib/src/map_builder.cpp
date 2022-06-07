/* ============================================================================================================================ *//**
 * @file       map_builder.cpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Saturday, 2nd April 2022 5:39:15 pm
 * @modified   Wednesday, 25th May 2022 4:59:26 pm
 * @project    engineering-thesis
 * @brief      Definition of the poi_map::MapBuilder class implementing routines focusing on managing an PoI map at runtime
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

/* =========================================================== Includes =========================================================== */

// System includes
#include <exception>
#include <utility>
// Private includes
#include "poi_map/map_builder.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace poi_map {

/* ======================================================== Static methods ======================================================== */

MapBuilderConstructor MapBuilder::make_constructor() {
    return MapBuilderConstructor{};
}

/* ========================================================= Public ctors ========================================================= */

MapBuilder::MapBuilder(
    std::size_t markers_limit,
    std::unique_ptr<TransformationStrategy> transformation_strategy,
    std::unique_ptr<MatchingStrategy> matching_strategy,
    std::unique_ptr<ScoreUpdateStrategy> score_update_strategy,
    std::unique_ptr<PositionUpdateStrategy> position_update_strategy,
    std::unique_ptr<DiscardingStrategy> discarding_strategy
) :
    transformation_strategy { std::move(transformation_strategy) },
    matching_strategy { std::move(matching_strategy) },
    score_update_strategy { std::move(score_update_strategy) },
    position_update_strategy { std::move(position_update_strategy) },
    discarding_strategy { std::move(discarding_strategy) }
{
    // Check if all strategies has been given
    bool valid_strategies_given = 
        (this->transformation_strategy != nullptr)   and
        (this->matching_strategy != nullptr)         and
        (this->score_update_strategy != nullptr) and
        (this->position_update_strategy != nullptr)  and
        (this->discarding_strategy != nullptr);

    // If not, throw error
    if(not valid_strategies_given)
        throw std::runtime_error{ "'poi_map::MapBuilder' object cannot be created with non-initialized dependencies" };

    // Reserve memory for the map
    map.reserve(markers_limit);
}

/* ======================================================== Public methods ======================================================== */

void MapBuilder::update(const std::vector<poi_map_msgs::msg::Marker>& markers) {

    // Inform strategies that the new portion of markers is processed
    transformation_strategy->reset();
    matching_strategy->reset();
    score_update_strategy->reset();
    position_update_strategy->reset();
    discarding_strategy->reset();

    // Iterate over incoming markers
    for(auto &raw_marker : markers) {

        // Transform incoming marker
        auto marker_transformed = transformation_strategy->transform(raw_marker, markers);
        // Check if marker should be processed
        if(not marker_transformed.has_value())
            continue;

        // If so, get reference to the marker
        auto &marker = *marker_transformed;

        // Try to find the corresponding marker in the current PoI map
        auto corresponding_marker_in_map = std::find_if( map.begin(), map.end(),
            [this, &marker](const auto &m) { return matching_strategy->match(m, marker); }
        );

        // If the corresponding marker has been found
        if(corresponding_marker_in_map != map.end()) {

            // Update score fo the marker
            corresponding_marker_in_map->score = score_update_strategy->update(*corresponding_marker_in_map, marker);
            // Update position fo the marker
            corresponding_marker_in_map->position = position_update_strategy->update(*corresponding_marker_in_map, marker);

        // If the corresponding marker has not been found
        } else {

            // If a new marker fits into the map
            if(map.size() < map.capacity()) {

                // Add a new marker in the map
                map.push_back(marker);
                // Reset marker's score
                map.back().score = 0;
                
            }

        }

    }

    // Discard out-of-date markers
    discarding_strategy->discard(map);
    
}


const std::vector<poi_map_msgs::msg::Marker> &MapBuilder::get_map() const {
    return map;
}


void MapBuilder::set_map(const std::vector<poi_map_msgs::msg::Marker> &map) {
    this->map = map;
}

/* ======================================================= Constructor class ====================================================== */

MapBuilderConstructor &MapBuilderConstructor::markers_limit(std::size_t limit) {
    this->markers_limit_ = limit;
    return *this;
}


MapBuilderConstructor &MapBuilderConstructor::transformation_strategy(std::unique_ptr<TransformationStrategy> strategy) {
    this->transformation_strategy_ = std::move(strategy);
    return *this;
}


MapBuilderConstructor &MapBuilderConstructor::matching_strategy(std::unique_ptr<MatchingStrategy> strategy) {
    this->matching_strategy_ = std::move(strategy);
    return *this;
}


MapBuilderConstructor &MapBuilderConstructor::score_update_strategy(std::unique_ptr<ScoreUpdateStrategy> strategy) {
    this->score_update_strategy_ = std::move(strategy);
    return *this;
}


MapBuilderConstructor &MapBuilderConstructor::position_update_strategy(std::unique_ptr<PositionUpdateStrategy> strategy) {
    this->position_update_strategy_ = std::move(strategy);
    return *this;
}


MapBuilderConstructor &MapBuilderConstructor::discarding_strategy(std::unique_ptr<DiscardingStrategy> strategy) {
    this->discarding_strategy_ = std::move(strategy);
    return *this;
}


MapBuilder MapBuilderConstructor::operator*() {
    
    // Check if all parameters has been set
    bool properly_initialized =
        markers_limit_.has_value()             and
        (transformation_strategy_ != nullptr)  and
        (matching_strategy_ != nullptr)        and
        (score_update_strategy_ != nullptr)    and
        (position_update_strategy_ != nullptr) and
        (discarding_strategy_ != nullptr);

    // Throw error if not
    if(not properly_initialized)
        throw std::runtime_error{ "Cannot generate poi_map::MapBuilder with lacking parameters" };

    // Create the builder object
    return MapBuilder(
        *markers_limit_,
        std::move(transformation_strategy_),
        std::move(matching_strategy_),
        std::move(score_update_strategy_),
        std::move(position_update_strategy_),
        std::move(discarding_strategy_)
    );

}

/* ================================================================================================================================ */

} // End namespace poi_map

/* ================================================================================================================================ */
