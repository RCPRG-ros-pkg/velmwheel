# ====================================================================================================================================
# @file       node.py
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Thursday, 7th April 2022 5:18:37 am
# @modified   Wednesday, 25th May 2022 4:44:01 pm
# @project    engineering-thesis
# @brief      Set of helper functions utilized by launchfiles across velmwheel_slam package
#    
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Imports ============================================================ #

from ament_index_python.packages import get_package_share_path
from velmwheel_launch.node import generate_component_launch_description
from velmwheel_model.params import ROBOT_NAME

# ============================================================= Launch ============================================================= #

# ---------------------------------------------------------------------------------------
# @brief Generates LaunchDescription object for the SLAM toolbox component node of the
#   ecosystem of the WUT Velmwheel robot
#
# @param executable
#    name of the executable inside the slam_toolbox package
# @param default_config_file (optional, default: None)
#    name of the default configuration file for the node; if @c None given the 
#    confgiguation-file launch parameters is not added and the configuration file is
#    not passed to the node; this may be absolute or relative to the @p package root
# ---------------------------------------------------------------------------------------
def generate_slam_component_launch_description(
    executable,
    default_config_file = None
):
    return generate_component_launch_description(
        
        # Executable to be run
        package = 'slam_toolbox',
        executable = executable,
        name = 'slam',

        # Default configuation file
        default_config_file = 
            str(get_package_share_path('slam_toolbox') / 'config' / default_config_file)
                if default_config_file is not None else None,

        # Common parameters for all configruation of the SLAM (overwrite config file)
        params = [

            { 'odom_frame':   'odom'                       },
            { 'map_frame':    'map'                        },
            { 'base_frame':   f'{ROBOT_NAME}'              },
            { 'scan_topic':   f'/{ROBOT_NAME}/lidars/scan' },
            
        ],

        # Common remaps for all configruation of the SLAM
        remaps = [

            # Outputs
            ( '/map',                              f'/{ROBOT_NAME}/slam/map',                ),
            ( '/map_metadata',                     f'/{ROBOT_NAME}/slam/map_metadata'        ),
            ( '/slam_toolbox/graph_visualization', f'/{ROBOT_NAME}/slam/graph_visualization' ),
            ( '/slam_toolbox/scan_visualization',  f'/{ROBOT_NAME}/slam/scan_visualization'  ),
            
        ]
        
    )
