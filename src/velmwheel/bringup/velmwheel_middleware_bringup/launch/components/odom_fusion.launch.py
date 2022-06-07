# ====================================================================================================================================
# @file       odom_fusion.launch.py
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Thursday, 7th April 2022 6:50:10 am
# @modified   Wednesday, 25th May 2022 4:44:01 pm
# @project    engineering-thesis
# @brief      Launchfile running middleware of the WUT Velmwheel robot's `odom_fusion` module
#    
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Imports ============================================================ #

# Launch imports
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.substitutions import PythonExpression
DeclareLaunchArgument
# Private imports
from launch_common.config import get_config_source
from launch_common.includes import get_launch_source
from launch_common.arguments import declare_launch_argument

# ============================================================ Arguments =========================================================== #

# Launch argument: whether to run 'custom' or 'original' implementation fo `robot_localization`
declare_type_config_description, type_config = declare_launch_argument({
    'name':          'odom_fusion_type',
    'default_value': 'custom',
    'description':   'Whether to run "custom" or "original" implementation fo `robot_localization`',
    'choices':       [ 'custom', 'original' ]
})

# ============================================================ Constants =========================================================== #

# Name of the component
component_name = 'odom_fusion'

# ===================================================== Components descriptors ===================================================== #

# Odometry fusion node
component_description = {

    # Source file
    'launch_description_source': get_launch_source(f'velmwheel_{component_name}', f'{component_name}.launch.py'),
    # Arguments
    'launch_arguments': {

        # Configuration file
        f'{component_name}_config': PythonExpression([
            
            # Custom-implementation config file
            "'", get_config_source('velmwheel_middleware_bringup', 'odom_fusion_custom.yaml'), "'",
            # Condition
            " if '", type_config, "' == 'custom' else ", 
            # Original-implementation config file
            "'", get_config_source('velmwheel_middleware_bringup', 'odom_fusion_original.yaml'), "'",

        ]),

        # Implementation mode
        'odom_fusion_type': type_config
        
    }.items()
    
}

# =========================================================== Description ========================================================== #

# Description of the launch
launch_description = [
    
    # Arguments
    DeclareLaunchArgument(**declare_type_config_description),

    # Includes
    IncludeLaunchDescription( **component_description )

]

# ================================================================================================================================== #

def generate_launch_description():
    return LaunchDescription(launch_description)
