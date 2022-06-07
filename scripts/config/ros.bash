# ====================================================================================================================================
# @ Filename: ros2_install.bash
# @ Author: Krzysztof Pierczyk
# @ Create Time: 2021-01-02 14:53:45
# @ Modified time: 2021-01-02 14:53:47
# @ Source: https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Install-Debians/
# @ Description:
#
#     Setups environment and installs ROS2 Foxy distribution along with recommended tools
#     
# @ Note: Before running this script `sourceMe.bash` (under project's home directory) should be sourced by the calling terminal
# @ Note: Script was tested on Ubuntu 20.0.4 LTS system
# @ Note: This script should be rather source than executed
# ====================================================================================================================================

# Setup path to vcstool
export PATH=$PATH:/home/$USER/.local/bin

# Source ROS setup script
source /opt/ros/$PROJECT_ROS_DISTRO/setup.bash
