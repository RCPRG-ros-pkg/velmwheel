# ====================================================================================================================================
# @file       MarkersStamped.msg
# @author     Wojtek Dudek
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Thursday, 31st March 2022 12:18:42 pm
# @modified   Wednesday, 25th May 2022 4:36:51 pm
# @project    engineering-thesis
# @brief      Auxiliary ROS message type describing a list of PoI-merkers
#    
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# Messages' header info
std_msgs/Header header
# List of points consituting the marker (coresponds to @a points field in the visualization_msgs/Marker message)
poi_map_msgs/Marker[] markers
