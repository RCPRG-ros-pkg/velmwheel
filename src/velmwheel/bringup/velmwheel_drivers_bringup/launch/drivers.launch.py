# ====================================================================================================================================
# @file       drivers.launch.py
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Sunday, 6th March 2022 7:04:33 pm
# @modified   Tuesday, 14th June 2022 4:16:58 pm
# @project    engineering-thesis
# @brief      Launchfile running WUT Velmwheel low-level drivers
#    
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Imports ============================================================ #

# Launch imports
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
# Private imports
from launch_common.includes import get_launch_source

# =========================================================== Description ========================================================== #

# Description of the launch
launch_description = [
    
    IncludeLaunchDescription( launch_description_source=get_launch_source('velmwheel_drivers_bringup', f'components/laser_driver.launch.py')    ),
    IncludeLaunchDescription( launch_description_source=get_launch_source('velmwheel_drivers_bringup', f'components/ethercat_driver.launch.py') ),

]

# ================================================================================================================================== #

def generate_launch_description():
    return LaunchDescription(launch_description)
