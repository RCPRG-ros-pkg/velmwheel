# ====================================================================================================================================
# @file       install.bash
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Friday, 13th May 2022 5:23:30 pm
# @modified   Monday, 18th July 2022 5:22:12 pm
# @project    engineering-thesis
# @brief      Checks for dependencancies of the project in the system and isntalls lacking ones
# 
# 
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# Install basic dependencies
$PROJECT_HOME/scripts/install/dependencies.bash
# Install buildtools 
$PROJECT_HOME/scripts/install/buildtools.bash
# Install Hilscher's sources 
$PROJECT_HOME/scripts/install/cifx.bash
# Install ROS
$PROJECT_HOME/scripts/install/ros.bash
# Install external projects
$PROJECT_HOME/scripts/install/extern.bash
