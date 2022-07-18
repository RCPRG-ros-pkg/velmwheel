# ====================================================================================================================================
# @file       ros.bash
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Monday, 20th June 2022 1:03:56 pm
# @modified   Wednesday, 22nd June 2022 12:17:04 pm
# @project    engineering-thesis
# @brief      Setups environment and installs ROS2 Foxy distribution along with recommended tools
# 
# 
# @note Before running this script `source_me.bash` (under project's home directory) should be sourced by the calling terminal
# @note Script was tested on Ubuntu 22.0.4 LTS system
# @note This script should be rather source than executed
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# Setup path to vcstool
export PATH=$PATH:/home/$USER/.local/bin

# Source ROS setup script
source /opt/ros/$PROJECT_ROS_DISTRO/setup.bash
