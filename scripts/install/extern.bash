# ====================================================================================================================================
# @file       extern.bash
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Monday, 18th July 2022 4:50:09 pm
# @modified   Monday, 18th July 2022 4:55:47 pm
# @project    engineering-thesis
# @brief      Setups environment elements related to external dependencies of the project
#
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# Ignore ros-common subdependency of the scan-tools project (ros-common itself is dependency of the project)
touch $PROJECT_HOME/extern/ros/scan-tools/extern/ros-common/COLCON_IGNORE
