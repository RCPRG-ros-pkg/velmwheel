# ====================================================================================================================================
# @file       base_controller.launch.py
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Friday, 4th March 2022 5:57:12 pm
# @modified   Wednesday, 25th May 2022 4:44:01 pm
# @project    engineering-thesis
# @brief      Launchfile for the base controllers for the the WUT Velmwheel robot's driveline
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
        package = 'velmwheel_base_controller',
        executable = 'velmwheel_base_controller',
        name = 'base_controller',
        # By default always ON
        default_on = 'true'
        
    )

# ================================================================================================================================== #
