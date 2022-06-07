# ====================================================================================================================================
# @file       poi_map_builder.launch.py
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Friday, 4th March 2022 5:57:12 pm
# @modified   Wednesday, 25th May 2022 4:44:01 pm
# @project    engineering-thesis
# @brief      Launchfile for the PoI (Points of Interest) map builder submodule for the the WUT Velmwheel robot's driveline
#    
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Imports ============================================================ #

from velmwheel_model.params import ROBOT_NAME
from velmwheel_launch.node import generate_component_launch_description

# ============================================================= Launch ============================================================= #

def generate_launch_description():
    return generate_component_launch_description(
        
        # Executable to be run
        package = 'velmwheel_poi_map_builder',
        executable = 'velmwheel_poi_map_builder',
        name = 'poi_map_builder',

        # Default configuation file
        default_config_file = "config/poi_map_builder.yaml",

        # Paramas overwrites
        params = [

            # Set fixed frames names
            { 'fixed_frame': 'map'       },
            { 'robot_frame': 'velmwheel' }
            
        ],

        # Remapps
        remaps = [

            # Inputs
            ( 'cloud', f'/{ROBOT_NAME}/lidars/cloud/combined' ),
            # Outputs
            ( 'points_of_interest', f'/{ROBOT_NAME}/markers'                     ),
            ( 'map',                f'/{ROBOT_NAME}/markers_map'                 ),
            ( 'visualization',      f'/{ROBOT_NAME}/markers_map/visualization'   ),
            ( 'change_mode',        f'/{ROBOT_NAME}/poi_map_builder/change_mode' ),
            ( 'save_map',           f'/{ROBOT_NAME}/poi_map_builder/save_map'    ),
            ( 'load_map',           f'/{ROBOT_NAME}/poi_map_builder/load_map'    )
        ]

    )

# ================================================================================================================================== #
