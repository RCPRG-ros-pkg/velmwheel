# ====================================================================================================================================
# @file       sick_laser_driver.launch.py
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Friday, 4th March 2022 5:57:12 pm
# @modified   Thursday, 7th July 2022 3:24:52 pm
# @project    engineering-thesis
# @brief      Launchfile starting producer's drivers of SICK LIDAR sensors preconfigured for WUT Velmwheel robot
#    
#    
# @note Launchfile has been based on the <sick_scan>/launch/sick_lms_1xx.launch launch file
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Imports ============================================================ #

# ROS includes
from launch import LaunchDescription
# Private includes
from velmwheel_model.params import ROBOT_NAME
from velmwheel_launch.node import generate_component_launch_descriptor

# ============================================================== Nodes ============================================================= #

def lidar_descriptor(name, frame_id, config_file, condition_arg):
    
    """Produces description of the driver of one of LIDAR drivers"""
    
    return generate_component_launch_descriptor(
        
        # Executable to be run
        package = 'velmwheel_laser_driver',
        executable = 'velmwheel_laser_driver',
        name = name,
        
        # Default configuation file
        default_config_file = f"config/{config_file}",

        # Additional arguments
        params = [

            # Driver's preconfigured configuration
            { 'scanner_type': 'sick_lms_1xx'      },
            { 'port': '2112'                      },
            { 'use_binary_protocol': True         },
            { 'intensity_resolution_16bit': False },
            { 'min_ang': -2.35619                 },
            { 'max_ang': 2.35619                  },
            { 'range_max': 25.0                   },
            { 'timelimit': 5                      },
            { 'intensity': True                   },
            { 'frame_id': frame_id                },
        ],

        # Remap I/O topics
        remappings = [

            # Remap output topic
            ( 'cloud', f'/{ROBOT_NAME}/lidars/cloud' ),

        ],

    )

# Nodes' descriptions
left_driver_node_description  = lidar_descriptor( 'lidar_l', 'lidar_l_core', 'sick/left_laser_driver.yaml'  )
right_driver_node_description = lidar_descriptor( 'lidar_r', 'lidar_r_core', 'sick/right_laser_driver.yaml' )

# ======================================================= Launch description ======================================================= #

launch_description = [

    # Nodes
    *left_driver_node_description,
    *right_driver_node_description,

]

# ================================================================================================================================== #

def generate_launch_description():
    return LaunchDescription(launch_description)