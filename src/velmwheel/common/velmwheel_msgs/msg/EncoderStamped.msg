# ====================================================================================================================================
# @file       EncoderStamped.msg
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Thursday, 28th April 2022 7:17:24 pm
# @modified   Wednesday, 25th May 2022 4:44:01 pm
# @project    engineering-thesis
# @brief      Interface representing set of measurements from the robot's wheel encoder with header
#    
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# Header informations of the measurements
std_msgs/Header header
# Actual measurements
velmwheel_msgs/Encoder encoder
