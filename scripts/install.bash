# ====================================================================================================================================
# @ Filename: dependancies.bash
# @ Author: Krzysztof Pierczyk
# @ Create Time: 2021-04-19 21:08:30
# @ Modified time: 2021-04-19 21:08:37
# @ Description: 
#    
#    Checks for dependencancies of the project in the system and isntalls lacking ones
#
# ====================================================================================================================================

# Install basic dependencies
$PROJECT_HOME/scripts/install/dependencies.bash
# Install buildtools 
$PROJECT_HOME/scripts/install/buildtools.bash
# Install Hilscher's sources 
$PROJECT_HOME/scripts/install/hilscher.bash
# Install ROS
$PROJECT_HOME/scripts/install/ros.bash
