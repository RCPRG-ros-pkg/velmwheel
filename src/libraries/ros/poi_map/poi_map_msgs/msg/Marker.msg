# ====================================================================================================================================
# @file       Marker.msg
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Thursday, 31st March 2022 4:46:45 pm
# @modified   Wednesday, 25th May 2022 4:35:57 pm
# @project    engineering-thesis
# @brief      Auxiliary ROS message type describing a PoI (Point of Interest) marker (i.e. a 2D point with a matching score value)
#
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# Position of the point on the 2D plain
poi_map_msgs/Point2D position
# Score of the point
float64 score
