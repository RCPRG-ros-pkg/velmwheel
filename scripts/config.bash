# ====================================================================================================================================
# @file       config.bash
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Friday, 13th May 2022 5:23:30 pm
# @modified   Thursday, 16th June 2022 12:27:52 pm
# @project    engineering-thesis
# @brief      Checks for dependencancies of the project in the system and isntalls lacking ones. Sources scripts providing definitions
#             and functionalities used in the project
#
# @note Before running this script `source_me.bash` (under project's home directory) should be sourced by the calling terminal
# @note Script was tested on Ubuntu 20.0.4 LTS system
# @note This script should be rather source than executed
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# Check whether `source_me.bash` was sourced
if [  -z "${PROJECT_HOME}" ]; then
    echo "[ERROR] Run 'source source_me.bash' in the root directory of the project to source this script"
fi

# Configure CIFX tools
bash $PROJECT_HOME/scripts/config/cifx.bash
# Install colcon tool
source $PROJECT_HOME/scripts/config/colcon.bash
# Configure ROS
source $PROJECT_HOME/scripts/config/ros.bash
