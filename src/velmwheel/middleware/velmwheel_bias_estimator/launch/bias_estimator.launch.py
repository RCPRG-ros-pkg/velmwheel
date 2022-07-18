# ====================================================================================================================================
# @file       bias_estimator.launch.py
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Friday, 4th March 2022 5:57:12 pm
# @modified   Monday, 18th July 2022 6:21:02 pm
# @project    engineering-thesis
# @brief      Launchfile for the bias estimator submodule for the the WUT Velmwheel robot's driveline
#    
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Imports ============================================================ #

from velmwheel_launch.node import generate_component_launch_description

# ============================================================= Launch ============================================================= #

def generate_launch_description():
    return generate_component_launch_description(
        
        # Executable to be run
        package = 'velmwheel_bias_estimator',
        executable = 'velmwheel_bias_estimator',
        name = 'bias_estimator',

        # Default configuation file
        default_config_file = "config/bias_estimator.yaml",

    )

# ================================================================================================================================== #
