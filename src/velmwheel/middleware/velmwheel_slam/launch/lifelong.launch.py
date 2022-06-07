# ====================================================================================================================================
# @file     lifelong.launch.py
# @author   Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date     Friday, 4th March 2022 5:57:12 pm
# @modified   Thursday, 7th April 2022 5:25:34 pm
# @project  engineering-thesis
# @brief
#    
#    Launchfile running the lifelong SLAM agent from the SLAM toolbox package           
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
        executable = 'lifelong_slam_toolbox_node',
        # Default configuation file
        default_config_file = 'mapper_params_lifelong.yaml'

    )

# ================================================================================================================================== #
