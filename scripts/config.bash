# ====================================================================================================================================
# @ Filename: dependancies.bash
# @ Author: Krzysztof Pierczyk
# @ Create Time: 2021-04-19 21:08:30
# @ Modified time: 2021-04-19 21:08:37
# @ Description: 
#    
#    Checks for dependencancies of the project in the system and isntalls lacking ones. Sources scripts providing definitions
#    and functionalities used in the project
#    
# @ Note: Before running this script `source_me.bash` (under project's home directory) should be sourced by the calling terminal
# @ Note: Script was tested on Ubuntu 20.0.4 LTS system
# @ Note: This script should be rather source than executed
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
