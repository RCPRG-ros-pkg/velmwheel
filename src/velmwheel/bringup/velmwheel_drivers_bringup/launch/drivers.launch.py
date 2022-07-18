# ====================================================================================================================================
# @file       drivers.launch.py
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Sunday, 6th March 2022 7:04:33 pm
# @modified   Friday, 15th July 2022 2:46:01 pm
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
from launch_common.models import get_urdf_source

# ======================================================= Robot-related nodes ====================================================== #

# Robot state and joints publisher
state_publisher = {

    # Source file
    'launch_description_source': get_launch_source('velmwheel_launch', 'state_publisher.launch.py'),

    # Launch Parameters
    'launch_arguments': {
        'urdf_path': get_urdf_source('velmwheel_model', 'velmwheel.urdf.xacro'),
        'use_sim_time': 'false',
    }.items(),
    
}

# =========================================================== Description ========================================================== #

# Description of the launch
launch_description = [
    
    # State publisher
    IncludeLaunchDescription( **state_publisher ),
    # Drivers
    IncludeLaunchDescription( launch_description_source=get_launch_source('velmwheel_drivers_bringup', f'components/laser_driver.launch.py')    ),
    IncludeLaunchDescription( launch_description_source=get_launch_source('velmwheel_drivers_bringup', f'components/ethercat_driver.launch.py') ),

]

# ================================================================================================================================== #

def generate_launch_description():
    return LaunchDescription(launch_description)
