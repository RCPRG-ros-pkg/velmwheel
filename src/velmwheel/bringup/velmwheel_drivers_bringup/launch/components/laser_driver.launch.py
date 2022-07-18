# ====================================================================================================================================
# @file       laser_driver.launch.py
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Thursday, 7th April 2022 6:50:10 am
# @modified   Thursday, 7th July 2022 3:10:47 pm
# @project    engineering-thesis
# @brief      Launchfile running middleware of the WUT Velmwheel robot's `laser_driver` module
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
component_name = 'laser_driver'

# ===================================================== Components descriptors ===================================================== #

# Laser driver nodes
component_description = {

    # Source file
    'launch_description_source': get_launch_source(f'velmwheel_{component_name}', f'{component_name}.launch.py'),
    # Arguments
    'launch_arguments': {

        # Configuration file
        'lidar_l_config':  get_config_source('velmwheel_drivers_bringup', 'left_laser_driver.yaml' ),
        'lidar_r_config': get_config_source('velmwheel_drivers_bringup', 'right_laser_driver.yaml'),
        # Hard nodes requirements
        'lidar_l_required' : 'true',
        'lidar_r_required' : 'true',
        
    }.items()
}

# =========================================================== Description ========================================================== #

# Description of the launch
launch_description = [
    
    # Includes
    IncludeLaunchDescription(**component_description)

]

# ================================================================================================================================== #

def generate_launch_description():
    return LaunchDescription(launch_description)
