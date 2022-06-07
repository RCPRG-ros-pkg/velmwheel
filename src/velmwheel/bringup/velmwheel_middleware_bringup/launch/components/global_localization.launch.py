# ====================================================================================================================================
# @file       global_localization.launch.py
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Thursday, 7th April 2022 6:50:10 am
# @modified   Wednesday, 25th May 2022 4:44:01 pm
# @project    engineering-thesis
# @brief      Launchfile running middleware of the WUT Velmwheel robot's `global_localization` module
#    
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Imports ============================================================ #

# Launch imports
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
# Private imports
from launch_common.config import get_config_source
from launch_common.includes import get_launch_source

# ============================================================ Constants =========================================================== #

# Name of the component
component_name = 'global_localization'

# ===================================================== Components descriptors ===================================================== #

# Global localization node
component_description = {

    # Source file
    'launch_description_source': get_launch_source(f'velmwheel_{component_name}', f'{component_name}.launch.py'),
    # Arguments
    'launch_arguments': {

        # Configuration file
        f'{component_name}_config': get_config_source('velmwheel_middleware_bringup', 'global_localization.yaml') ,
        
    }.items()
}

# =========================================================== Description ========================================================== #

# Description of the launch
launch_description = [
    
    # Includes
    IncludeLaunchDescription( **component_description )

]

# ================================================================================================================================== #

def generate_launch_description():
    return LaunchDescription(launch_description)
