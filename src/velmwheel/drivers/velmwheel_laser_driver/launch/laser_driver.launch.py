# ====================================================================================================================================
# @file       base_controller.launch.py
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Friday, 4th March 2022 5:57:12 pm
# @modified   Friday, 15th July 2022 5:03:52 pm
# @project    engineering-thesis
# @brief      Launchfile starting producer's drivers of SICK LIDAR sensors preconfigured for WUT Velmwheel robot
#    
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Imports ============================================================ #

# ROS includes
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
# Private includes
from launch_common.arguments import declare_launch_argument
from velmwheel_launch.node import generate_component_launch_descriptor

# ======================================================== Launch arguments ======================================================== #

# Launch argument: output mode of the LIDAR drivers
declare_laser_output_mode_config_description, laser_output_mode_config = declare_launch_argument({
    'name':          'laser_output_mode',
    'default_value': 'separate',
    'description':   'Publishing mode of the node indicating what topics it publishes to (@see lidar_gazebo.hpp)',
    'choices':       [ 'separate', 'common', 'both' ]
})

# ============================================================== Nodes ============================================================= #

def lidar_descriptor(name, config_file):
    
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

            # Publishing mode
            { 'publishing_mode': laser_output_mode_config },
            # Reference frame
            { 'reference_frame': f'{name}_scan' },
        ],

    )

# Nodes' descriptions
left_driver_node_description  = lidar_descriptor( 'lidar_l', 'left_laser_driver.yaml'  )
right_driver_node_description = lidar_descriptor( 'lidar_r', 'right_laser_driver.yaml' )

# ======================================================= Launch description ======================================================= #

launch_description = [
    
    # Arguments
    DeclareLaunchArgument(**declare_laser_output_mode_config_description),
    # Nodes
    *left_driver_node_description,
    *right_driver_node_description,

]

# ================================================================================================================================== #

def generate_launch_description():
    return LaunchDescription(launch_description)
