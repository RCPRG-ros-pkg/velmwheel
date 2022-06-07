# ====================================================================================================================================
# @file     online_async.launch.py
# @author   Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date     Friday, 4th March 2022 5:57:12 pm
# @modified   Thursday, 7th April 2022 5:26:21 pm
# @project  engineering-thesis
# @brief
#    
#    Launchfile running the online asynchronous SLAM agent from the SLAM toolbox package           
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
        executable = 'async_slam_toolbox_node',
        # Default configuation file
        default_config_file = 'mapper_params_online_async.yaml'

    )

# ================================================================================================================================== #
