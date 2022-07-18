# ====================================================================================================================================
# @file       colcon.bash
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Monday, 20th June 2022 1:03:56 pm
# @modified   Wednesday, 22nd June 2022 12:17:36 pm
# @project    engineering-thesis
# @brief      Installs colcon build system used along with ROS2 packages
# 
# 
# @note Before running this script `source_me.bash` (under project's home directory) should be sourced by the calling terminal
# @note Script was tested on Ubuntu 22.04 LTS system
# @note This script should be rather source than executed
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# Source additional script for colcon autocompletion
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
# Source colcon_cd
source /usr/share/colcon_cd/function/colcon_cd.sh
