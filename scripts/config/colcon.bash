# ====================================================================================================================================
# @ Filename: colcon.bash
# @ Author: Krzysztof Pierczyk
# @ Create Time: 2021-04-19 21:23:07
# @ Modified time: 2021-04-19 21:23:24
# @ Source: https://colcon.readthedocs.io/en/released/user/installation.html
# @ Description:
#
#     Installs colcon build system used along with ROS2 packages
#     
# @ Note: Before running this script `sourceMe.bash` (under project's home directory) should be sourced by the calling terminal
# @ Note: Script was tested on Ubuntu 20.0.4 LTS system
# @ Note: This script should be rather source than executed
# ====================================================================================================================================

# Source additional script for colcon autocompletion
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
# Source colcon_cd
source /usr/share/colcon_cd/function/colcon_cd.sh
