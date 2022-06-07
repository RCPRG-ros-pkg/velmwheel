/* ============================================================================================================================ *//**
 * @file       poi_map_strategies.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Saturday, 2nd April 2022 1:13:19 pm
 * @modified   Thursday, 26th May 2022 12:04:57 am
 * @project    engineering-thesis
 * @brief      Declarations of classes providing strategies for selecting points of interest (POI) in the environment of the WUT Velmwheel
 *             robot and building POI map
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __VELMWHEEL_POI_MAP_STRATEGIES_H__
#define __VELMWHEEL_POI_MAP_STRATEGIES_H__

/* =========================================================== Includes =========================================================== */

// Interface includes
#include "poi_map_msgs/msg/point2_d.hpp"
#include "poi_map_msgs/msg/marker.hpp"
// Private includes
#include "poi_map/strategies.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace velmwheel::points_of_interest {

/* ================================================== Data-processing strategies ================================================== */

/**
 * @brief An abstract base class implementing preprocessing algorithm for incoming 
 *    environment-scan data
 * 
 */
struct DataFilteringStrategy {
    
    /**
     * @brief Destroy the DataFilteringStrategy object
     */
    virtual ~DataFilteringStrategy() = default;

    /**
     * @brief Filters incoming @p data
     * 
     * @param[inout] data 
     *    data to be filtered
     * @returns 
     *    filtered data
     */
    virtual std::vector<poi_map_msgs::msg::Marker> 
    filter(const std::vector<poi_map_msgs::msg::Marker> &data) 
    { 
        return data;
    };
    
};


/**
 * @brief An preprocessing algorithm class filtering input data discarding points which
 *    are too loosele packed
 */
class DenseDataFilter : public DataFilteringStrategy {
public:

    /**
     * @brief Constructs the DenseDataFilter object
     * 
     * @param max_distance 
     *    maximal acceptable distance between subsequent points
     */
    DenseDataFilter(double max_distance);

    /**
     * @brief Destroy the DenseDataFilter object
     */
    virtual ~DenseDataFilter() = default;

    /**
     * @brief Filters incoming @p data removing too-densly packed points
     * 
     * @param data 
     *    data to be filtered
     * @returns 
     *    filtered data
     */
    std::vector<poi_map_msgs::msg::Marker> 
    filter(const std::vector<poi_map_msgs::msg::Marker> &data) override;

private:

    /// Maximal acceptable distance between subsequent points
    const double max_distance;

};

/* ==================================================== Map-building strategies =================================================== */

/**
 * @brief Map builder's transformation strategy used to choose only those of incoming
 *    markers to be matched with in-map markers that are placed in the center of 
 *    a region of interest (ROI). ROI is defined as a contiguous section of the scanned
 *    sorroundings where level of intensity is higher than a preconfigured threshold.
 */
class RegionOfInterestFilter : public poi_map::TransformationStrategy {
public:

    /**
     * @brief Destroy the RegionOfInterestFilter object
     * 
     * @param roi_threshold 
     *    intensity threshold defining boundary of the ROI
     */
    RegionOfInterestFilter(double roi_threshold);

    /**
     * @brief Destroy the RegionOfInterestFilter object
     * 
     */
    virtual ~RegionOfInterestFilter() = default;

    /**
     * @brief Checks if the @p incoming_marker lies in the ROI. If so, it tries to transform
     *    it into the point being a centre of the ROI. Otherwise it marks marker to not be
     *    processed
     * 
     * @param[in] incoming_marker
     *    incoming marker being processed
     * @param[in] all_incoming_markers
     *    all incoming markers in current batch
     * @returns 
     *    @retval transformed version of the marker
     *    @retval empty optional if the marker should not be processed
     */
    std::optional<poi_map_msgs::msg::Marker> transform(
        const poi_map_msgs::msg::Marker& incoming_marker,
        const std::vector<poi_map_msgs::msg::Marker>& all_incoming_markers
    ) override;


    /**
     * @brief Resets state of the filter in preparation for the new scan
     */
    void reset() override;

private:

    /// Intensity threshold defining boundary of the ROI
    const double roi_threshold;

    /// Index of the currently processed marker in the batch
    std::size_t processed_marker_idx { 0 };
    
    /// Boolean flag indicating whether last processed point lied in the ROI
    bool last_marker_in_roi { false };
    /// Index of the last marker opening the ROI
    std::size_t last_roi_start_idx { 0 };
    
};

/**
 * @brief Points-matching strategy of the Builder class. Matches points based if
 *    the rectilinear distance between them doesn't cross a preconfigured value
 */
class RectilinearMatcher : public poi_map::MatchingStrategy {
public:

    /**
     * @brief Construct a new RectilinearMatcher object with the
     *    given @p threshold
     * 
     * @param threshold 
     *    rectilinear distance threshold matching two points on the 2D plane
     */
    RectilinearMatcher(const poi_map_msgs::msg::Point2D &threshold);

    /**
     * @brief Destroy the RectilinearMatcher object
     * 
     */
    virtual ~RectilinearMatcher() = default;

    /**
     * @param[in] local_marker
     *    local marker
     * @param[in] incoming_maarker
     *    incoming marker
     * @returns 
     *    @retval @c true if given points match each other
     *    @retval @c false otherwise
     */
    bool match(
        const poi_map_msgs::msg::Marker& local_marker,
        const poi_map_msgs::msg::Marker& incoming_maarker
    ) override;

private:

    /// Distance threshold used for points-matching
    const poi_map_msgs::msg::Point2D threshold;

};


/**
 * @brief Intensity-update strategy of the Builder class. Calculates new intensity
 *    of the marker by incrementing it's old intensity by @c 1 up to the limit
 */
class IntensityIncrementer : public poi_map::ScoreUpdateStrategy {
public:

    /**
     * @brief Construct a new IntensityUpdateStrategy object
     * 
     * @param limit 
     *    maximum value
     */
    IntensityIncrementer(double limit);

    /**
     * @brief Destroy the IntensityIncrementer object
     * 
     */
    virtual ~IntensityIncrementer() = default;

    /**
     * @brief Calculates new intensity of the marker by incrementing it's old intensity by @c 1
     * 
     * @param[in] local_marker
     *    local marker
     * @param[in] incoming_maarker
     *    incoming marker
     * @returns 
     *    intensity of the @p local_marker incremented by @c 1
     */
    double update(
        const poi_map_msgs::msg::Marker& local_marker,
        const poi_map_msgs::msg::Marker& incoming_marker
    ) override;

private:

    /// Limit of the marker's intensity
    const double limit;

};


/**
 * @brief Position-update strategy of the Builder class. Calculates new positon
 *    of the marker by moving it's old position half a way toward the incoming marker
 */
struct HalfawayMover : public poi_map::PositionUpdateStrategy {

    /**
     * @brief Construct a new HalfawayMover object
     */
    HalfawayMover() = default;

    /**
     * @brief Destroy the HalfawayMover object
     * 
     */
    virtual ~HalfawayMover() = default;
    
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
    poi_map_msgs::msg::Point2D update(
        const poi_map_msgs::msg::Marker& local_marker,
        const poi_map_msgs::msg::Marker& incoming_maarker
    ) override;
    
};

/* ================================================================================================================================ */

} // End namespace velmwheel::points_of_interest

#endif
