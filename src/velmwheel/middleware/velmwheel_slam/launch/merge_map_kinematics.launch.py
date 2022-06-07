# ====================================================================================================================================
# @file       merge_map_kinematics.launch.py
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Friday, 4th March 2022 5:57:12 pm
# @modified   Wednesday, 25th May 2022 11:16:52 pm
# @project    engineering-thesis
# @brief      Launches @ref merge_map_kinematics node from the SLAM toolbox
# 
# 
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Imports ============================================================ #

from velmwheel_slam.node import generate_slam_component_launch_description

# ============================================================= Launch ============================================================= #

def generate_launch_description():
    return generate_slam_component_launch_description(
        
        # Executable to be run
        executable = 'merge_maps_kinematic'

    )

# ================================================================================================================================== #
