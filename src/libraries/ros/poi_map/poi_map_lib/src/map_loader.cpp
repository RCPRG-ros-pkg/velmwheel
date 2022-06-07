/* ============================================================================================================================ *//**
 * @file       map_loader.cpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 31st March 2022 2:07:55 pm
 * @modified   Wednesday, 25th May 2022 5:00:05 pm
 * @project    engineering-thesis
 * @brief      Definition of the poi_map::MapLoader class responsible for loading/storing an PoI map to/from the drive
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

/* =========================================================== Includes =========================================================== */

// Boost includes
#include <boost/foreach.hpp>
// Private includes
#include "poi_map/map_loader.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace poi_map {

/* ======================================================== Helper classes ======================================================== */

std::string MapLoader::MarkerSubelementPathBuilder::build(std::size_t marker_id, const char *subelement_name) {

    stringbuilder.clear();

    // Build the path
    stringbuilder
        << ELEMENT_NAMES::MAP                       << "."
        << ELEMENT_NAMES::POINT_PREFIX << marker_id << "."
        << subelement_name;

    return stringbuilder.str();
}

/* ============================================================= Ctors ============================================================ */

MapLoader::MapLoader(double store_score_threshold) :
    store_score_threshold { store_score_threshold }
{ }

/* ======================================================== Loading methods ======================================================= */

std::vector<poi_map_msgs::msg::Marker> MapLoader::load_map(const std::string &file) const {

    boost::property_tree::ptree xml_tree;
    
    // Load XML file
	boost::property_tree::read_xml(file, xml_tree);

    std::vector<poi_map_msgs::msg::Marker> ret;
    
    // Reserve memory for PoI markers
    ret.reserve(xml_tree.count(ELEMENT_NAMES::MAP));

    // Iterate over points description and place them into the resulting vector
	BOOST_FOREACH(const boost::property_tree::ptree::value_type &value, xml_tree.get_child(ELEMENT_NAMES::MAP)) {

        // Create a new marker in the map
        ret.emplace_back();
        // Get reference to the constructed marker
        auto &marker = ret.back();
        // Configure the marker
        marker.position.x = value.second.get<double>( ELEMENT_NAMES::X_COORDINATE );
        marker.position.y = value.second.get<double>( ELEMENT_NAMES::Y_COORDINATE );
        marker.score  = value.second.get<double>( ELEMENT_NAMES::INTENSITY    );

    }
    
    return ret;
}

/* ======================================================== Helper methods ======================================================== */

template<typename Getter>
void MapLoader::save_map(const std::string &file, std::size_t size, Getter &&getter) const {

    MarkerSubelementPathBuilder namebuilder;
    boost::property_tree::ptree xml_tree;
    
    // Iterate over markers in the map to build the XML tree
    for (unsigned i = 0; i < size; i++ ) {

        // If score of the marker is higher than the threshold value, store it in the resulting tree
    	if (getter(i, MarkerComponent::Score) > store_score_threshold) {

            // Store the coordinates in the stree
	        xml_tree.put(namebuilder.build(i, ELEMENT_NAMES::X_COORDINATE ), getter(i, MarkerComponent::PositionX));
	        xml_tree.put(namebuilder.build(i, ELEMENT_NAMES::Y_COORDINATE ), getter(i, MarkerComponent::PositionY));
	        xml_tree.put(namebuilder.build(i, ELEMENT_NAMES::INTENSITY    ), getter(i, MarkerComponent::Score));
         
    	}

    }

    // Write down the XML tree into the file
    boost::property_tree::write_xml(
        file, xml_tree,
        std::locale(),
        boost::property_tree::xml_writer_make_settings<boost::property_tree::ptree::key_type>(' ', INTENDATION_SPACES)
    );
    
}

/* ======================================================== Storing methods ======================================================= */

void MapLoader::set_store_score_threshold(double threshold) {
    store_score_threshold = threshold;
}

void MapLoader::save_map(const std::string &file, const std::vector<poi_map_msgs::msg::Marker> &map) const {
    save_map(file, map.size(), [&map](std::size_t index, MarkerComponent component) -> double {
        switch(component) {
            case MarkerComponent::PositionX: return map.at(index).position.x;
            case MarkerComponent::PositionY: return map.at(index).position.y;
            case MarkerComponent::Score:     return map.at(index).score;
            default:                         return 0;
        }
    });
}


void MapLoader::save_map(const std::string &file, const visualization_msgs::msg::Marker &map) const {
    save_map(file, map.points.size(), [&map](std::size_t index, MarkerComponent component) -> double {
        switch(component) {
            case MarkerComponent::PositionX: return map.points.at(index).x;
            case MarkerComponent::PositionY: return map.points.at(index).y;
            case MarkerComponent::Score:     return map.points.at(index).z;
            default:                         return 0;
        }
    });
}

/* ================================================================================================================================ */

} // End namespace poi_map

/* ================================================================================================================================ */
