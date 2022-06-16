# ====================================================================================================================================
# @file       install.bash
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Friday, 13th May 2022 5:23:30 pm
# @modified   Thursday, 16th June 2022 12:29:29 pm
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
$PROJECT_HOME/scripts/install/hilscher.bash
# Install ROS
$PROJECT_HOME/scripts/install/ros.bash
