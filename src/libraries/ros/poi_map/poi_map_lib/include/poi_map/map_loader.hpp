/* ============================================================================================================================ *//**
 * @file       loader.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 31st March 2022 1:37:51 pm
 * @modified   Wednesday, 25th May 2022 4:58:11 pm
 * @project    engineering-thesis
 * @brief      Declaration of the poi_map::Loader class responsible for loading/storing an PoI map to/from the drive
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __POI_MAP_LOADER_H__
#define __POI_MAP_LOADER_H__

/* =========================================================== Includes =========================================================== */

// System includes
#include <string>
#include <vector>
// ROS includes
#include "visualization_msgs/msg/marker.hpp"
// Boost includes
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
// Private includes
#include "poi_map_msgs/msg/marker.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace poi_map {

/* ============================================================= Class ============================================================ */

/**
 * @brief Class responsible for loading/storing an PoI map to/from the drive in shape of 
 *    an XML files
 * @details An PoI map is is represented as an arbitrary list of 3D points describing 2D
 *    position of the point as first two components and it's score as a third
 *    component. The map can be represented also (mainly for visualization purpose) as 
 *    a visualization_msgs::msg::Marker structure. In such a case the components of each marker
 *    is stored in the @a points member of the structure.
 */
class MapLoader {
public:

    /**
     * @brief Names of the XML elements describing the PoI map
     */
    struct ELEMENT_NAMES {

        /// Name of the top element
        static constexpr auto MAP { "map" };
        /// Prefix of the name of the element descibing a single PoI point
        static constexpr auto POINT_PREFIX { "marker_" };
        /// Name of the element descibing an X coordinate of the point
        static constexpr auto X_COORDINATE { "x" };
        /// Name of the element descibing an X coordinate of the point
        static constexpr auto Y_COORDINATE { "y" };
        /// Name of the element descibing a score of the point
        static constexpr auto INTENSITY { "score" };

    };

    /// Number of spaces of intendation in the stored XML files
    static constexpr auto INTENDATION_SPACES = 4U;

public:

    /**
     * @brief Construct a new MapLoader object
     * 
     * @param store_score_threshold
     *   (exclusive) threshold value determining minimal score of the marker to
     *   not being discarded when storing map into XML file
     */
    MapLoader(double store_score_threshold);

    /**
     * @brief Destructs a MapLoader object releasing all acquired resources
     */
    ~MapLoader() = default;

public:

    /**
     * @brief Loads an PoI map from the XML file named @p file
     * 
     * @param[in] file
     *    name of the file to load data from 
     * @returns 
     *    loaded map as a vectr of points
     */
    std::vector<poi_map_msgs::msg::Marker> load_map(const std::string &file) const;

public:

    /**
     * @brief Sets (exclusive) threshold value determining minimal score of the marker to
     *   not being discarded when storing map into XML file
     * 
     * @param threshold
     *   threshold to be set
     */
    void set_store_score_threshold(double threshold);

public:

    /**
     * @brief Saves PoI map given as a @p map vector to the XML file named
     *    @p file
     * 
     * @param file 
     *    name of the output XML file 
     * @param map 
     *    map to be saved
     */
    void save_map(const std::string &file, const std::vector<poi_map_msgs::msg::Marker> &map) const;

    /**
     * @brief Saves PoI map given as a @p map visualization_msgs::msg::Marker structure
     *    to the XML file named @p file
     * 
     * @param file 
     *    name of the output XML file 
     * @param map 
     *    map to be saved
     */
    void save_map(const std::string &file, const visualization_msgs::msg::Marker &map) const;    

private:

    /**
     * @brief Helper builder class creating XML paths to the subelement of marker's
     *    description with the given parameters
     * 
     */
    class MarkerSubelementPathBuilder {
    public:

        /**
         * @brief Build path to the @p subelement_name of the marker point 
         *    stored in the 'ELEMENT_NAMES::POINT_PREFIX<marker_id>' subelement
         *    of the map element
         * 
         * @param marker_id 
         *    index of the marker
         * @param coordinate_name 
         *    name of the target subelement of the marker
         */
        std::string build(std::size_t marker_id, const char *subelement_name);

    private:

        // String builder used to compose names
        std::ostringstream stringbuilder;

    };

private:

    /**
     * @brief Auxiliary enumartion of components of a single marker
     */
    enum class MarkerComponent {
        PositionX,
        PositionY,
        Score
    };

    /**
     * @brief Saves PoI map consisting of @p size markers to the XML file named
     *     @p file basing on the marker's description acquired from the @p getter 
     *     functor
     * 
     * @param file 
     *    name of the output XML file 
     * @param size 
     *    number of markers in the map
     * @param getter 
     *    callback functor with the double(std::size_t index, MarkerComponent component)
     *    signature returning value of the @p component of the marker with indes @p index
     */
    template<typename Getter>
    void save_map(const std::string &file, std::size_t size, Getter &&getter) const;  

private:

    /// (Exclusive) Threshold value determining minimal score of the marker to not being discarded when storing map into XML file
    double store_score_threshold;

};

/* ================================================================================================================================ */

} // End namespace poi_map

/* ================================================================================================================================ */

#endif
