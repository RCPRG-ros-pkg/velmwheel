# ====================================================================================================================================
# @file       MarkersMap.msg
# @author     Wojtek Dudek
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Thursday, 31st March 2022 12:18:42 pm
# @modified   Wednesday, 25th May 2022 4:36:29 pm
# @project    engineering-thesis
# @brief      Auxiliary ROS message type describing a list of PoI-markers representing an PoI map
#    
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# Messages' header info
std_msgs/Header header
# List of points consituting the marker (coresponds to @a points field in the visualization_msgs/Marker message)
poi_map_msgs/Marker[] markers
# Boolean flag set to 'true' when the published map has been 'realod' (e.g. form the file) since the last publication
bool reloaded