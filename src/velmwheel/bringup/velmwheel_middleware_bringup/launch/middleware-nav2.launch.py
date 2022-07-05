# ====================================================================================================================================
# @file       launch.py
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Thursday, 7th April 2022 6:50:10 am
# @modified   Wednesday, 25th May 2022 4:44:01 pm
# @project    engineering-thesis
# @brief      Launchfile running middleware of the WUT Velmwheel robot's control stack
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
    
    IncludeLaunchDescription( launch_description_source=get_launch_source('velmwheel_middleware_bringup', f'components/base_controller.launch.py')     ),
    IncludeLaunchDescription( launch_description_source=get_launch_source('velmwheel_middleware_bringup', f'components/laser_odom.launch.py')          ),
    IncludeLaunchDescription( launch_description_source=get_launch_source('velmwheel_middleware_bringup', f'components/odom_fusion.launch.py')         ),
    IncludeLaunchDescription( launch_description_source=get_launch_source('velmwheel_middleware_bringup', f'components/bias_estimator.launch.py')      ),
    IncludeLaunchDescription( launch_description_source=get_launch_source('velmwheel_middleware_bringup', f'components/localization.launch.py')                ),
    IncludeLaunchDescription( launch_description_source=get_launch_source('velmwheel_middleware_bringup', f'components/poi_map_builder.launch.py')     ),
    IncludeLaunchDescription( launch_description_source=get_launch_source('velmwheel_middleware_bringup', f'components/global_localization.launch.py') ),

]

# ================================================================================================================================== #

def generate_launch_description():
    return LaunchDescription(launch_description)
