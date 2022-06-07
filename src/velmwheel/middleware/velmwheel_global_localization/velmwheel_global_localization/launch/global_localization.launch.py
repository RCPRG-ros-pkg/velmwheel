# ====================================================================================================================================
# @file       globa_localization.launch.py
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Friday, 4th March 2022 5:57:12 pm
# @modified   Wednesday, 25th May 2022 4:44:01 pm
# @project    engineering-thesis
# @brief      Launchfile for the global localization submodule for the the WUT Velmwheel robot's driveline
#    
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Imports ============================================================ #

from velmwheel_launch.node import generate_component_launch_description
from velmwheel_model.params import ROBOT_NAME

# ============================================================= Launch ============================================================= #

def generate_launch_description():
    return generate_component_launch_description(
        
        # Executable to be run
        package = 'velmwheel_global_localization',
        executable = 'velmwheel_global_localization',
        name = 'globa_localization',

        # Default configuation file
        default_config_file = "config/globa_localization.yaml",

        # Remapps
        remaps = [

            # Inputs
            ( 'odom', f'/{ROBOT_NAME}/odom/filtered' ),
            # Outputs
            ( 'pose',        f'/{ROBOT_NAME}/global_localization/pose'        ),
            ( 'change_mode', f'/{ROBOT_NAME}/global_localization/change_mode' )

        ]
        
    )

# ================================================================================================================================== #
